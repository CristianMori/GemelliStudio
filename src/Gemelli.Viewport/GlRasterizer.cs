using System.Numerics;
using Silk.NET.Maths;
using Silk.NET.OpenGL;
using Silk.NET.Windowing;

namespace Gemelli.Viewport;

/// <summary>
/// An offscreen OpenGL rasterizer for the navigation viewport: uploads the loaded geometry once, then each
/// frame draws every mesh (flat Lambert shading + ground grid) into an FBO at the requested size and reads
/// the pixels back as RGBA. Owns a hidden-window GL context — construct and use it from a single thread.
/// </summary>
public sealed unsafe class GlRasterizer : IDisposable
{
    private readonly IWindow _window;
    private readonly GL _gl;
    private int _w, _h;
    private uint _fbo, _colorTex, _depthRbo;
    private uint _prog, _gridProg;
    private uint _gridVao, _gridVbo;
    private int _gridVerts;
    private uint _groundVao, _groundVbo;
    private readonly List<(uint Vao, uint Vbo, int Count, Vector3 Color, string? Body)> _meshes = new();
    private byte[] _pixels = [];

    /// <summary>
    /// Creates a hidden-window GL 3.3 core context on the calling thread, links the mesh + grid shader programs,
    /// builds the offscreen FBO and the static grid/ground buffers, and enables depth test + back-face culling.
    /// </summary>
    public GlRasterizer(int width, int height)
    {
        _w = width; _h = height;
        var opts = WindowOptions.Default with
        {
            IsVisible = false,
            Size = new Vector2D<int>(Math.Max(1, width), Math.Max(1, height)),
            API = new GraphicsAPI(ContextAPI.OpenGL, ContextProfile.Core, ContextFlags.Default, new APIVersion(3, 3)),
        };
        _window = Window.Create(opts);
        _window.Initialize();                 // creates + makes current the GL context on this thread
        _gl = _window.CreateOpenGL();

        _prog = Link(VertSrc, FragSrc);
        _gridProg = Link(GridVertSrc, GridFragSrc);
        BuildFbo();
        BuildGrid();

        _gl.Enable(EnableCap.DepthTest);
        _gl.Enable(EnableCap.CullFace);
        _gl.CullFace(TriangleFace.Back);
    }

    /// <summary>
    /// Replaces the GPU mesh set: frees the previous VAOs/VBOs, then uploads each mesh's interleaved
    /// position+normal buffer (6 floats/vertex) and records its draw count, color, and owning body path.
    /// </summary>
    public void Upload(IReadOnlyList<RenderMesh> meshes)
    {
        foreach (var (vao, vbo, _, _, _) in _meshes) { _gl.DeleteVertexArray(vao); _gl.DeleteBuffer(vbo); }
        _meshes.Clear();
        foreach (RenderMesh m in meshes)
        {
            if (m.Vertices.Length == 0) continue;
            uint vao = _gl.GenVertexArray();
            _gl.BindVertexArray(vao);
            uint vbo = _gl.GenBuffer();
            _gl.BindBuffer(BufferTargetARB.ArrayBuffer, vbo);
            fixed (float* p = m.Vertices)
                _gl.BufferData(BufferTargetARB.ArrayBuffer, (nuint)(m.Vertices.Length * sizeof(float)), p, BufferUsageARB.StaticDraw);
            _gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), (void*)0);                  // loc 0: position
            _gl.EnableVertexAttribArray(0);
            _gl.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), (void*)(3 * sizeof(float))); // loc 1: normal
            _gl.EnableVertexAttribArray(1);
            _meshes.Add((vao, vbo, m.Vertices.Length / 6, m.Color, m.BodyPath));
        }
        _gl.BindVertexArray(0);
    }

    /// <summary>Reallocates the FBO (color texture + depth buffer) to a new size; no-op if unchanged or invalid.</summary>
    public void Resize(int width, int height)
    {
        if (width == _w && height == _h || width <= 0 || height <= 0) return;
        _w = width; _h = height;
        _gl.DeleteFramebuffer(_fbo); _gl.DeleteTexture(_colorTex); _gl.DeleteRenderbuffer(_depthRbo);
        BuildFbo();
    }

    /// <summary>
    /// Render one frame. <paramref name="modelLookup"/> maps a body path to its live world transform
    /// (System.Numerics row-vector); null → identity (static geometry). Returns RGBA8 (w*h*4), top-left origin.
    /// </summary>
    public byte[] Render(Matrix4x4 view, Matrix4x4 proj, Func<string, Matrix4x4?> modelLookup)
    {
        _gl.BindFramebuffer(FramebufferTarget.Framebuffer, _fbo);
        _gl.Viewport(0, 0, (uint)_w, (uint)_h);
        _gl.ClearColor(0.10f, 0.11f, 0.13f, 1f);
        _gl.Clear((uint)(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit));

        Matrix4x4 vp = view * proj;
        Vector3 lightDir = Vector3.Normalize(new Vector3(0.35f, 0.45f, 0.82f));

        // Ground quad (lit, dark) — gives the scene a floor without the scene's huge ground mesh.
        _gl.UseProgram(_prog);
        _gl.Uniform3(_gl.GetUniformLocation(_prog, "uLightDir"), lightDir.X, lightDir.Y, lightDir.Z);
        SetMat(_prog, "uModel", Matrix4x4.Identity);
        SetMat(_prog, "uMVP", vp);
        _gl.Uniform3(_gl.GetUniformLocation(_prog, "uColor"), 0.16f, 0.17f, 0.20f);
        _gl.BindVertexArray(_groundVao);
        _gl.DrawArrays(PrimitiveType.Triangles, 0, 6);

        // Ground grid lines on top.
        _gl.UseProgram(_gridProg);
        SetMat(_gridProg, "uVP", vp);
        _gl.BindVertexArray(_gridVao);
        _gl.DrawArrays(PrimitiveType.Lines, 0, (uint)_gridVerts);

        // Meshes.
        _gl.UseProgram(_prog);
        foreach (var (vao, _, count, color, body) in _meshes)
        {
            Matrix4x4 model = body is not null ? modelLookup(body) ?? Matrix4x4.Identity : Matrix4x4.Identity;
            SetMat(_prog, "uModel", model);
            SetMat(_prog, "uMVP", model * vp);
            _gl.Uniform3(_gl.GetUniformLocation(_prog, "uColor"), color.X, color.Y, color.Z);
            _gl.BindVertexArray(vao);
            _gl.DrawArrays(PrimitiveType.Triangles, 0, (uint)count);
        }
        _gl.BindVertexArray(0);

        // Readback.
        int need = _w * _h * 4;
        if (_pixels.Length != need) _pixels = new byte[need];
        _gl.PixelStore(PixelStoreParameter.PackAlignment, 1);
        fixed (byte* p = _pixels)
            _gl.ReadPixels(0, 0, (uint)_w, (uint)_h, PixelFormat.Rgba, PixelType.UnsignedByte, p);
        _gl.BindFramebuffer(FramebufferTarget.Framebuffer, 0);
        FlipY(_pixels, _w, _h); // GL origin is bottom-left; callers expect top-left
        return _pixels;
    }

    // ---- setup helpers ----

    // Build the offscreen target: RGBA8 color texture (nearest, no mips) + 24-bit depth renderbuffer, both at _w×_h.
    private void BuildFbo()
    {
        _fbo = _gl.GenFramebuffer();
        _gl.BindFramebuffer(FramebufferTarget.Framebuffer, _fbo);
        _colorTex = _gl.GenTexture();
        _gl.BindTexture(TextureTarget.Texture2D, _colorTex);
        _gl.TexImage2D(TextureTarget.Texture2D, 0, InternalFormat.Rgba8, (uint)_w, (uint)_h, 0, PixelFormat.Rgba, PixelType.UnsignedByte, null);
        _gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMinFilter, (int)GLEnum.Nearest);
        _gl.TexParameter(TextureTarget.Texture2D, TextureParameterName.TextureMagFilter, (int)GLEnum.Nearest);
        _gl.FramebufferTexture2D(FramebufferTarget.Framebuffer, FramebufferAttachment.ColorAttachment0, TextureTarget.Texture2D, _colorTex, 0);
        _depthRbo = _gl.GenRenderbuffer();
        _gl.BindRenderbuffer(RenderbufferTarget.Renderbuffer, _depthRbo);
        _gl.RenderbufferStorage(RenderbufferTarget.Renderbuffer, InternalFormat.DepthComponent24, (uint)_w, (uint)_h);
        _gl.FramebufferRenderbuffer(FramebufferTarget.Framebuffer, FramebufferAttachment.DepthAttachment, RenderbufferTarget.Renderbuffer, _depthRbo);
        _gl.BindFramebuffer(FramebufferTarget.Framebuffer, 0);
    }

    // Build the static reference floor: a line-list grid (XY plane, z=0) plus a filled quad just beneath it.
    private void BuildGrid()
    {
        var v = new List<float>();
        const int N = 20; const float S = 1f; // 1 m grid, ±20 m
        for (int i = -N; i <= N; i++)
        {
            v.AddRange([i * S, -N * S, 0f]); v.AddRange([i * S, N * S, 0f]);
            v.AddRange([-N * S, i * S, 0f]); v.AddRange([N * S, i * S, 0f]);
        }
        _gridVerts = v.Count / 3;
        _gridVao = _gl.GenVertexArray();
        _gl.BindVertexArray(_gridVao);
        _gridVbo = _gl.GenBuffer();
        _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _gridVbo);
        float[] arr = v.ToArray();
        fixed (float* p = arr)
            _gl.BufferData(BufferTargetARB.ArrayBuffer, (nuint)(arr.Length * sizeof(float)), p, BufferUsageARB.StaticDraw);
        _gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 3 * sizeof(float), (void*)0);
        _gl.EnableVertexAttribArray(0);

        // Ground quad (pos+normal, normal +Z), just below the grid to avoid z-fighting. Lit by the mesh shader.
        const float G = (float)N * S;
        float[] ground =
        {
            -G, -G, -0.004f, 0, 0, 1,  G, -G, -0.004f, 0, 0, 1,  G, G, -0.004f, 0, 0, 1,
            -G, -G, -0.004f, 0, 0, 1,  G,  G, -0.004f, 0, 0, 1, -G, G, -0.004f, 0, 0, 1,
        };
        _groundVao = _gl.GenVertexArray();
        _gl.BindVertexArray(_groundVao);
        _groundVbo = _gl.GenBuffer();
        _gl.BindBuffer(BufferTargetARB.ArrayBuffer, _groundVbo);
        fixed (float* p = ground)
            _gl.BufferData(BufferTargetARB.ArrayBuffer, (nuint)(ground.Length * sizeof(float)), p, BufferUsageARB.StaticDraw);
        _gl.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), (void*)0);
        _gl.EnableVertexAttribArray(0);
        _gl.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        _gl.EnableVertexAttribArray(1);
        _gl.BindVertexArray(0);
    }

    private void SetMat(uint prog, string name, Matrix4x4 m)
    {
        // System.Numerics is row-major/row-vector; upload transposed so GLSL (column-vector) matches.
        int loc = _gl.GetUniformLocation(prog, name);
        float* f = stackalloc float[16]
        {
            m.M11, m.M12, m.M13, m.M14, m.M21, m.M22, m.M23, m.M24,
            m.M31, m.M32, m.M33, m.M34, m.M41, m.M42, m.M43, m.M44,
        };
        _gl.UniformMatrix4(loc, 1, false, f);
    }

    // Compile a vertex+fragment pair into a linked program, discard the shaders, throw on link failure.
    private uint Link(string vs, string fs)
    {
        uint v = Compile(ShaderType.VertexShader, vs), f = Compile(ShaderType.FragmentShader, fs);
        uint prog = _gl.CreateProgram();
        _gl.AttachShader(prog, v); _gl.AttachShader(prog, f); _gl.LinkProgram(prog);
        _gl.GetProgram(prog, ProgramPropertyARB.LinkStatus, out int ok);
        if (ok == 0) throw new InvalidOperationException("GL link failed: " + _gl.GetProgramInfoLog(prog));
        _gl.DeleteShader(v); _gl.DeleteShader(f);
        return prog;
    }

    // Compile one shader stage from GLSL source, throwing with the info log on error.
    private uint Compile(ShaderType type, string src)
    {
        uint s = _gl.CreateShader(type);
        _gl.ShaderSource(s, src);
        _gl.CompileShader(s);
        _gl.GetShader(s, ShaderParameterName.CompileStatus, out int ok);
        if (ok == 0) throw new InvalidOperationException($"GL {type} compile failed: " + _gl.GetShaderInfoLog(s));
        return s;
    }

    // In-place vertical flip of the RGBA buffer: swaps row y with row (h-1-y) to convert GL's bottom-left
    // origin to the top-left origin callers expect.
    private static void FlipY(byte[] px, int w, int h)
    {
        int stride = w * 4;
        var tmp = new byte[stride];
        for (int y = 0; y < h / 2; y++)
        {
            int top = y * stride, bot = (h - 1 - y) * stride;
            Array.Copy(px, top, tmp, 0, stride);
            Array.Copy(px, bot, px, top, stride);
            Array.Copy(tmp, 0, px, bot, stride);
        }
    }

    /// <summary>Frees every GL object (mesh/grid/ground buffers, FBO attachments, programs) and the window/context.</summary>
    public void Dispose()
    {
        try
        {
            foreach (var (vao, vbo, _, _, _) in _meshes) { _gl.DeleteVertexArray(vao); _gl.DeleteBuffer(vbo); }
            _gl.DeleteFramebuffer(_fbo); _gl.DeleteTexture(_colorTex); _gl.DeleteRenderbuffer(_depthRbo);
            _gl.DeleteVertexArray(_gridVao); _gl.DeleteBuffer(_gridVbo);
            _gl.DeleteVertexArray(_groundVao); _gl.DeleteBuffer(_groundVbo);
            _gl.DeleteProgram(_prog); _gl.DeleteProgram(_gridProg);
            _window.Dispose();
        }
        catch { }
    }

    // ---- shaders ----
    private const string VertSrc = """
        #version 330 core
        layout(location=0) in vec3 aPos;
        layout(location=1) in vec3 aNormal;
        uniform mat4 uMVP;
        uniform mat4 uModel;
        out vec3 vN;
        void main() {
            gl_Position = uMVP * vec4(aPos, 1.0);
            vN = mat3(uModel) * aNormal;
        }
        """;
    private const string FragSrc = """
        #version 330 core
        in vec3 vN;
        uniform vec3 uColor;
        uniform vec3 uLightDir;
        out vec4 frag;
        void main() {
            vec3 n = normalize(vN);
            vec3 L = normalize(uLightDir);
            // Hemisphere ambient (Z-up): sky tint on up-faces, cool shadow on down-faces.
            vec3 sky = vec3(0.58, 0.62, 0.70);
            vec3 grd = vec3(0.16, 0.17, 0.20);
            vec3 amb = mix(grd, sky, n.z * 0.5 + 0.5);
            float diff = max(dot(n, L), 0.0);
            vec3 col = uColor * (0.45 * amb + 0.75 * diff);
            col = pow(col, vec3(0.85));            // gentle gamma lift
            frag = vec4(col, 1.0);
        }
        """;
    private const string GridVertSrc = """
        #version 330 core
        layout(location=0) in vec3 aPos;
        uniform mat4 uVP;
        void main() { gl_Position = uVP * vec4(aPos, 1.0); }
        """;
    private const string GridFragSrc = """
        #version 330 core
        out vec4 frag;
        void main() { frag = vec4(0.27, 0.30, 0.36, 1.0); }
        """;
}
