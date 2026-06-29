using System.Runtime.InteropServices;

namespace Gemelli.Core.Control;

/// <summary>A snapshot of an Xbox controller's state (normalized axes, raw button bitmask).</summary>
public readonly record struct GamepadState(
    bool Connected,
    float LeftX, float LeftY, float RightX, float RightY,
    float LeftTrigger, float RightTrigger,
    ushort Buttons)
{
    // Common XInput button bits.
    public bool A => (Buttons & 0x1000) != 0;
    public bool B => (Buttons & 0x2000) != 0;
    public bool LeftBumper => (Buttons & 0x0100) != 0;
    public bool RightBumper => (Buttons & 0x0200) != 0;
}

/// <summary>Reads Xbox/XInput controllers via the Windows XInput API (no external dependencies).</summary>
public static class Gamepad
{
    [StructLayout(LayoutKind.Sequential)]
    private struct XInputGamepad
    {
        public ushort wButtons;
        public byte bLeftTrigger, bRightTrigger;
        public short sThumbLX, sThumbLY, sThumbRX, sThumbRY;
    }

    [StructLayout(LayoutKind.Sequential)]
    private struct XInputState
    {
        public uint dwPacketNumber;
        public XInputGamepad Gamepad;
    }

    [DllImport("xinput1_4.dll")]
    private static extern uint XInputGetState(uint dwUserIndex, out XInputState pState);

    private const uint ErrorSuccess = 0;
    private const float Deadzone = 0.18f;

    /// <summary>Reads controller <paramref name="index"/> (0–3). Returns a disconnected state on failure.</summary>
    public static GamepadState Read(uint index = 0)
    {
        if (!OperatingSystem.IsWindows()) return default;
        uint result;
        XInputState s;
        try { result = XInputGetState(index, out s); }
        catch (DllNotFoundException) { return default; }
        if (result != ErrorSuccess) return default;

        var g = s.Gamepad;
        return new GamepadState(
            Connected: true,
            LeftX: Axis(g.sThumbLX), LeftY: Axis(g.sThumbLY),
            RightX: Axis(g.sThumbRX), RightY: Axis(g.sThumbRY),
            LeftTrigger: g.bLeftTrigger / 255f, RightTrigger: g.bRightTrigger / 255f,
            Buttons: g.wButtons);
    }

    // Normalizes a raw thumbstick axis to [-1,1], zeroing the deadzone and rescaling past it.
    private static float Axis(short raw)
    {
        float v = raw / 32767f;
        if (v > 1f) v = 1f;
        if (MathF.Abs(v) < Deadzone) return 0f;
        // Rescale past the deadzone so motion starts smoothly from 0.
        return (v - MathF.CopySign(Deadzone, v)) / (1f - Deadzone);
    }
}
