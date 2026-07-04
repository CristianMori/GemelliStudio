using Gemelli.Fmi;
using Xunit;

namespace Gemelli.Tests;

/// <summary>
/// Tests for the FMI co-simulation layer. The FMU/SSP tests drive the REAL demo archives built by
/// <c>tools\build-fmus.ps1</c> (native win64 DLLs inside), so they skip when those artifacts are
/// absent; the schema test is self-contained (writes its own stage) but needs the USD.NET natives
/// that ship with the test output.
/// </summary>
public class FmiTests
{
    private static string RepoRoot
    {
        get
        {
            for (var dir = new DirectoryInfo(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
                if (File.Exists(Path.Combine(dir.FullName, "Gemelli.slnx"))) return dir.FullName;
            return AppContext.BaseDirectory;
        }
    }

    private static string FmiDir => Path.Combine(RepoRoot, "scenes", "fmi");

    // ---- FMI 2.0 host against a real FMU ----

    [SkippableFact]
    public void MotorDrive_Fmu_Ramps_To_Speed_Command_And_EStops()
    {
        string fmu = Path.Combine(FmiDir, "MotorDrive.fmu");
        Skip.IfNot(File.Exists(fmu), "Build the demo FMUs first: tools\\build-fmus.ps1");

        using var motor = new Fmu2Instance(fmu, "test-motor");
        Assert.Contains("speedCommand", motor.Variables.Keys);
        motor.Initialize(0.0, new Dictionary<string, double> { ["enable"] = 1, ["eStop"] = 0 });

        // Command 5 rad/s; the drive ramps at its acceleration parameter (8/s), so 1s is plenty.
        double t = 0;
        const double dt = 1.0 / 60.0;
        motor.SetReals(new Dictionary<string, double> { ["speedCommand"] = 5 });
        for (int i = 0; i < 60; i++) { motor.Step(t, dt); t += dt; }
        Assert.Equal(5.0, motor.GetReal("targetVelocity"), 0.2);

        // E-stop: the target must decay to zero regardless of the speed command.
        motor.SetReals(new Dictionary<string, double> { ["eStop"] = 1 });
        for (int i = 0; i < 120; i++) { motor.Step(t, dt); t += dt; }
        Assert.Equal(0.0, motor.GetReal("targetVelocity"), 0.05);
    }

    // ---- SSP host against the real conveyor archive (3 FMUs, internal wiring) ----

    [SkippableFact]
    public void Conveyor_Ssp_Wires_Operator_Inputs_Through_To_Zone_Velocities()
    {
        string ssp = Path.Combine(FmiDir, "conveyor_demo.ssp");
        Skip.IfNot(File.Exists(ssp), "Build the demo FMUs first: tools\\build-fmus.ps1");

        using var conveyor = new SspInstanceModel(ssp, "test-conveyor");
        Assert.Contains("operatorSpeed", conveyor.InputConnectors);
        Assert.Contains("targetVelocity0", conveyor.OutputConnectors);
        conveyor.Initialize(0.0);

        var running = new Dictionary<string, double>
        {
            ["operatorSpeed"] = 10, ["rejectSpeed"] = 10, ["enable"] = 1, ["eStop"] = 0, ["rawPresence"] = 0,
        };

        double t = 0;
        const double dt = 1.0 / 60.0;
        IReadOnlyDictionary<string, double> outputs = new Dictionary<string, double>();
        for (int i = 0; i < 120; i++) { outputs = conveyor.Step(running, t, dt); t += dt; }

        // Sensor + controller + five motor drives: every zone must be commanded to the belt speed.
        for (int zone = 0; zone < 5; zone++)
            Assert.Equal(10.0, outputs[$"targetVelocity{zone}"], 0.2);

        // E-stop propagates through the controller AND each motor drive inside the SSP.
        var stopped = new Dictionary<string, double>(running) { ["eStop"] = 1 };
        for (int i = 0; i < 180; i++) { outputs = conveyor.Step(stopped, t, dt); t += dt; }
        for (int zone = 0; zone < 5; zone++)
            Assert.Equal(0.0, outputs[$"targetVelocity{zone}"], 0.05);
    }

    // ---- USD-FMI schema parsing (self-contained stage) ----

    [Fact]
    public void FmiSchema_Parses_Instances_Mappings_And_Initial_Values()
    {
        string dir = Path.Combine(Path.GetTempPath(), "gemelli-fmi-schema-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(dir);
        string usd = Path.Combine(dir, "scene.usda");
        try
        {
            File.WriteAllText(usd, """
                #usda 1.0
                (
                    defaultPrim = "World"
                    upAxis = "Z"
                )

                def Xform "World"
                {
                    def Xform "Controls"
                    {
                        double3 xformOp:translate = (7, 8, 9)
                        uniform token[] xformOpOrder = ["xformOp:translate"]
                    }

                    def SspInstance "Line"
                    {
                        bool fmi:enabled = 1
                        asset fmi:ssp = @./line.ssp@

                        def FmuConnection "Panel"
                        {
                            rel fmi:targets = </World/Controls>

                            def FmuMapping "SpeedRead"
                            {
                                token fmi:direction = "input"
                                token fmi:fmuAttribute = "speed"
                                token fmi:usdAttribute = "xformOp:translate"
                                int2 fmi:usdMapping = (1, 1)
                            }

                            def FmuMapping "DriveWrite"
                            {
                                token fmi:direction = "output"
                                token fmi:fmuAttribute = "targetVelocity"
                                token fmi:usdAttribute = "drive:angular:physics:targetVelocity"
                                int2 fmi:usdMapping = (0, 0)
                            }
                        }
                    }

                    def FmuInstance "Disabled"
                    {
                        bool fmi:enabled = 0
                        asset fmi:fmu = @./other.fmu@
                    }
                }
                """);

            FmiSceneConfig? config = FmiSchema.Load(usd);

            Assert.NotNull(config);
            FmiInstanceConfig line = Assert.Single(config.Instances); // disabled instance excluded
            Assert.True(line.IsSsp);
            Assert.Equal("/World/Line", line.PrimPath);
            Assert.Equal(Path.Combine(dir, "line.ssp"), line.ArchivePath);

            FmiConnection panel = Assert.Single(line.Connections);
            Assert.Equal("/World/Controls", panel.TargetPath);
            Assert.Equal(2, panel.Mappings.Count);

            FmiMapping speed = panel.Mappings[0];
            Assert.True(speed.IsInput);
            Assert.Equal("speed", speed.FmuVariable);
            Assert.Equal(("xformOp:translate", 1, 1), (speed.UsdAttribute, speed.Offset, speed.Count));

            FmiMapping drive = panel.Mappings[1];
            Assert.False(drive.IsInput);
            Assert.Equal(FmiSchema.DriveTargetVelocity, drive.UsdAttribute);

            // The input attribute's initial value was captured for start values / per-step inputs.
            Assert.Equal([7.0, 8.0, 9.0], config.InitialAttributeValues["/World/Controls"]["xformOp:translate"]);
        }
        finally
        {
            try { Directory.Delete(dir, recursive: true); } catch { }
        }
    }

    [Fact]
    public void FmiSchema_Returns_Null_For_A_Scene_Without_Fmi_Prims()
    {
        string usd = Path.Combine(Path.GetTempPath(), "gemelli-fmi-none-" + Guid.NewGuid().ToString("N") + ".usda");
        File.WriteAllText(usd, "#usda 1.0\n\ndef Xform \"World\"\n{\n}\n");
        try
        {
            Assert.Null(FmiSchema.Load(usd));
        }
        finally
        {
            try { File.Delete(usd); } catch { }
        }
    }
}
