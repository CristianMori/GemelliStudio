using System.Collections.Concurrent;

namespace Gemelli.Core.Control;

/// <summary>
/// Process-wide keyboard state shared between the UI (which feeds key down/up events) and scripts
/// running on the sim thread (which query it). The host (Studio) calls <see cref="SetKey"/> from its
/// Avalonia key handlers; scripts call <see cref="IsDown(string)"/> (or use <see cref="KeyboardState"/>).
/// Hosts without a UI (headless/MCP) never feed keys, so queries simply return false there.
/// </summary>
public static class Keyboard
{
    private static readonly ConcurrentDictionary<string, byte> Down = new();

    /// <summary>Host hook: record a key as pressed/released. <paramref name="name"/> is an Avalonia
    /// <c>Key.ToString()</c> value ("W", "Left", "Space", "D1").</summary>
    public static void SetKey(string name, bool isDown)
    {
        string k = Normalize(name);
        if (isDown) Down[k] = 1; else Down.TryRemove(k, out _);
    }

    /// <summary>Host hook: clear all keys (e.g. when the window loses focus, so none get stuck).</summary>
    public static void Clear() => Down.Clear();

    /// <summary>True if the named key is currently held — e.g. "W", "X", "1", "Left", "Space", "Shift".</summary>
    public static bool IsDown(string key) => Down.ContainsKey(Normalize(key));

    /// <summary>True if the given character's key is held (case-insensitive).</summary>
    public static bool IsDown(char key) => IsDown(key.ToString());

    // Normalize so "D1".."D0" (Avalonia digit key names) and "1".."0" both resolve to the digit,
    // and casing/aliases are consistent.
    private static string Normalize(string name)
    {
        string s = name.Trim().ToUpperInvariant();
        if (s.Length == 2 && s[0] == 'D' && char.IsDigit(s[1])) return s[1].ToString(); // D1 -> 1
        return s switch { "CONTROL" => "CTRL", "ESCAPE" => "ESC", _ => s };
    }
}

/// <summary>
/// A keyboard snapshot exposing each key as a boolean (held = true), so a script reads keys directly:
/// <c>var kb = ReadKeyboard(); if (kb.W) ...</c>, <c>kb.X</c>, <c>kb.Space</c>, <c>kb.Left</c>, or the
/// general <c>kb["1"]</c>. Values reflect the live key state at access time.
/// </summary>
public readonly struct KeyboardState
{
    /// <summary>True if the named key is held (e.g. "1", "F5", "PageUp").</summary>
    public bool this[string key] => Keyboard.IsDown(key);

    /// <summary>True if the named key is held.</summary>
    public bool Down(string key) => Keyboard.IsDown(key);

    // Letter keys.
    public bool A => Keyboard.IsDown("A");
    public bool B => Keyboard.IsDown("B");
    public bool C => Keyboard.IsDown("C");
    public bool D => Keyboard.IsDown("D");
    public bool E => Keyboard.IsDown("E");
    public bool F => Keyboard.IsDown("F");
    public bool G => Keyboard.IsDown("G");
    public bool H => Keyboard.IsDown("H");
    public bool I => Keyboard.IsDown("I");
    public bool J => Keyboard.IsDown("J");
    public bool K => Keyboard.IsDown("K");
    public bool L => Keyboard.IsDown("L");
    public bool M => Keyboard.IsDown("M");
    public bool N => Keyboard.IsDown("N");
    public bool O => Keyboard.IsDown("O");
    public bool P => Keyboard.IsDown("P");
    public bool Q => Keyboard.IsDown("Q");
    public bool R => Keyboard.IsDown("R");
    public bool S => Keyboard.IsDown("S");
    public bool T => Keyboard.IsDown("T");
    public bool U => Keyboard.IsDown("U");
    public bool V => Keyboard.IsDown("V");
    public bool W => Keyboard.IsDown("W");
    public bool X => Keyboard.IsDown("X");
    public bool Y => Keyboard.IsDown("Y");
    public bool Z => Keyboard.IsDown("Z");

    // Common control keys.
    public bool Space => Keyboard.IsDown("Space");
    public bool Left => Keyboard.IsDown("Left");
    public bool Right => Keyboard.IsDown("Right");
    public bool Up => Keyboard.IsDown("Up");
    public bool Shift => Keyboard.IsDown("LeftShift") || Keyboard.IsDown("RightShift") || Keyboard.IsDown("Shift");
    public bool Enter => Keyboard.IsDown("Enter") || Keyboard.IsDown("Return");
    // Down arrow: use kb["Down"] (avoids colliding with the Down(string) method).
}
