using System;
using System.Globalization;
using System.Drawing;
using System.IO;
using System.Reflection;
using System.Threading;
using System.Windows.Forms;

namespace JDY31PidTuner
{
    internal static class TunerSmokeTests
    {
        private static int checks;

        private static NumericUpDown Number(PidTunerForm form, string name)
        {
            return (NumericUpDown)typeof(PidTunerForm).GetField(name,
                BindingFlags.NonPublic | BindingFlags.Instance).GetValue(form);
        }

        private static void Check(bool condition, string message)
        {
            if (!condition) throw new Exception(message);
            checks++;
        }

        private static T Field<T>(PidTunerForm form, string name)
        {
            return (T)typeof(PidTunerForm).GetField(name,
                BindingFlags.NonPublic | BindingFlags.Instance).GetValue(form);
        }

        private static object Call(PidTunerForm form, string name, params object[] arguments)
        {
            return typeof(PidTunerForm).GetMethod(name,
                BindingFlags.NonPublic | BindingFlags.Instance).Invoke(form, arguments);
        }

        [STAThread]
        private static void Main()
        {
            // Run this executable from a temporary directory, never beside live settings.
            string settings = Path.Combine(AppDomain.CurrentDomain.BaseDirectory,
                "pid_settings.ini");
            if (File.Exists(settings)) throw new Exception("Test directory must be empty.");
            Application.EnableVisualStyles();
            using (var form = new PidTunerForm())
            {
                string[] fields = { "verticalKp", "verticalKd", "velocityKp",
                    "velocityKi", "pitchMid", "targetSpeed", "turnKp", "turnKd", "targetTurn" };
                decimal[] steps = { 10M, 0.01M, 0.01M, 0.001M, 0.1M, 1M, 1M, 0.01M, 1M };
                for (int i = 0; i < fields.Length; i++)
                {
                    Check(Number(form, fields[i]).Increment == steps[i], fields[i] + " increment");
                    Check(Number(form, fields[i] + "Step").Value == steps[i], fields[i] + " step");
                }
                Check(Number(form, "velocityKp").Minimum == -20M, "Preserve velocity Kp minimum");
                Check(Number(form, "velocityKp").Maximum == 100M, "Expanded velocity Kp maximum");
                Check(Number(form, "targetSpeed").Value == 0M, "Target speed initial value");
                Check(Number(form, "targetSpeed").DecimalPlaces == 0, "Integer target speed");
                Check(Number(form, "targetSpeedStep").Minimum == 1M, "Integer speed step");
                Check(Number(form, "turnKp").Value == 20M, "Initial turn Kp");
                Check(Number(form, "turnKd").Value == 0M, "Initial turn feedback disabled");
                Check(Number(form, "targetTurn").Value == 0M, "Initial turn command");
                Check(Number(form, "targetTurn").DecimalPlaces == 0, "Integer turn command");
                Check(Number(form, "targetTurn").Parent == Number(form, "targetSpeed").Parent,
                    "Turn command alongside target speed");
                Check(((TableLayoutPanel)Number(form, "targetTurn").Parent).GetRow(Number(form, "targetTurn")) == 7,
                    "Turn command below target speed");
                Check(Number(form, "targetTurn").Minimum == -100M &&
                    Number(form, "targetTurn").Maximum == 100M, "Turn command bounds");
                string preview = Environment.GetEnvironmentVariable("JDY31_SMOKE_PREVIEW_PATH");
                if (!string.IsNullOrEmpty(preview))
                {
                    form.Opacity = 0;
                    form.ShowInTaskbar = false;
                    form.Show();
                    form.PerformLayout();
                    using (var bitmap = new Bitmap(form.Width, form.Height))
                    {
                        form.DrawToBitmap(bitmap, new Rectangle(Point.Empty, form.Size));
                        bitmap.Save(preview);
                    }
                    var steeringPage = (TabPage)Number(form, "turnKd").Parent.Parent;
                    ((TabControl)steeringPage.Parent).SelectedTab = steeringPage;
                    form.PerformLayout();
                    using (var bitmap = new Bitmap(form.Width, form.Height))
                    {
                        form.DrawToBitmap(bitmap, new Rectangle(Point.Empty, form.Size));
                        bitmap.Save(Path.Combine(Path.GetDirectoryName(preview), "steering-preview.png"));
                    }
                    var motionPage = (TabPage)Field<Button>(form, "motionForwardButton").Parent.Parent.Parent;
                    ((TabControl)motionPage.Parent).SelectedTab = motionPage;
                    form.PerformLayout();
                    using (var bitmap = new Bitmap(form.Width, form.Height))
                    {
                        form.DrawToBitmap(bitmap, new Rectangle(Point.Empty, form.Size));
                        bitmap.Save(Path.Combine(Path.GetDirectoryName(preview), "motion-preview.png"));
                    }
                    form.Hide();
                }
                Check(!Field<Button>(form, "motionForwardButton").Enabled &&
                    !Field<Button>(form, "motionStopButton").Enabled, "Motion disabled while disconnected");
                Check(Number(form, "motionSpeedValue").Value == 0M &&
                    Number(form, "motionTurnValue").Value == 0M, "Fixed values initially zero");
                Check(Number(form, "motionSpeedValue").Maximum == 1000M &&
                    Number(form, "motionTurnValue").Maximum == 100M, "Fixed value bounds");
                Call(form, "ToggleMotionTargets", 1, 0);
                Check(Field<object>(form, "activeMotionDirection").ToString() == "None",
                    "Zero magnitude cannot select a moving direction");
                Number(form, "motionSpeedValue").Value = 20M;
                Number(form, "motionTurnValue").Value = 7M;
                Check(Number(form, "targetSpeedStep").Value == 1M &&
                    Number(form, "targetTurnStep").Value == 1M, "Fixed values preserve parameter steps");
                Call(form, "ToggleMotionTargets", 1, 0);
                Check(Number(form, "targetSpeed").Value == 20M &&
                    Number(form, "targetTurn").Value == 0M, "First forward starts fixed speed");
                Check(Field<Button>(form, "motionForwardButton").BackColor == Color.FromArgb(44, 120, 210) &&
                    !Field<Button>(form, "motionForwardButton").UseVisualStyleBackColor, "Selected forward is highlighted");
                Call(form, "ToggleMotionTargets", 1, 0);
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == 0M, "Second forward stops both targets");
                Call(form, "ToggleMotionTargets", 1, 0);
                Call(form, "ToggleMotionTargets", 0, -1);
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == -7M, "Forward then right clears forward");
                Check(Field<object>(form, "activeMotionDirection").ToString() == "Right" &&
                    Field<Button>(form, "motionForwardButton").UseVisualStyleBackColor &&
                    !Field<Button>(form, "motionRightButton").UseVisualStyleBackColor, "Only right remains selected");
                Call(form, "ToggleMotionTargets", 0, -1);
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == 0M, "Second right stops both targets");
                int[,] directions = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
                decimal[] speeds = { 20M, -20M, 0M, 0M };
                decimal[] turns = { 0M, 0M, 7M, -7M };
                string[] names = { "Forward", "Backward", "Left", "Right" };
                for (int from = 0; from < 4; from++)
                {
                    for (int to = 0; to < 4; to++)
                    {
                        Call(form, "StopMotion");
                        Call(form, "ToggleMotionTargets", directions[from, 0], directions[from, 1]);
                        Call(form, "ToggleMotionTargets", directions[to, 0], directions[to, 1]);
                        decimal speed = from == to ? 0M : speeds[to];
                        decimal turn = from == to ? 0M : turns[to];
                        string selected = from == to ? "None" : names[to];
                        Check(Number(form, "targetSpeed").Value == speed &&
                            Number(form, "targetTurn").Value == turn &&
                            Field<object>(form, "activeMotionDirection").ToString() == selected,
                            "Exclusive toggle transition " + names[from] + " -> " + names[to]);
                        string[] sent = ((string)Call(form, "BuildPidFrame")).Trim().Split(',');
                        Check(sent.Length == 10 &&
                            sent[6] == speed.ToString(CultureInfo.InvariantCulture) &&
                            sent[9] == turn.ToString(CultureInfo.InvariantCulture),
                            "Wire targets for " + names[from] + " -> " + names[to]);
                    }
                }
                Call(form, "StopMotion");
                Call(form, "ToggleMotionTargets", 1, 0);
                Number(form, "motionSpeedValue").Value = 30M;
                Call(form, "ToggleMotionTargets", 1, 0);
                Check(Number(form, "targetSpeed").Value == 0M,
                    "Same direction still toggles off after magnitude edit");
                Call(form, "ToggleMotionTargets", 1, 0);
                Check(Number(form, "targetSpeed").Value == 30M, "Next activation uses new fixed magnitude");
                Number(form, "targetSpeed").Value = 23M;
                Number(form, "targetTurn").Value = 9M;
                Check(Field<object>(form, "activeMotionDirection").ToString() == "None",
                    "Manual target edits clear direction selection");
                Call(form, "ToggleMotionTargets", 0, -1);
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == -7M, "Direction selection clears manually entered mixed motion");
                Number(form, "motionSpeedValue").Value = 20M;
                Field<CheckBox>(form, "motionForwardReverse").Checked = true;
                Field<CheckBox>(form, "motionTurnReverse").Checked = true;
                Call(form, "ToggleMotionTargets", 1, 0);
                Check(Number(form, "targetSpeed").Value == -20M &&
                    Number(form, "targetTurn").Value == 0M, "Reversed forward remains exclusive");
                Call(form, "ToggleMotionTargets", 0, -1);
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == 7M, "Reversed right clears speed");
                Call(form, "ToggleMotionTargets", 0, -1);
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == 0M, "Reversed same-direction toggle stops");
                Field<CheckBox>(form, "motionForwardReverse").Checked = false;
                Field<CheckBox>(form, "motionTurnReverse").Checked = false;
                Number(form, "motionSpeedValue").Value = 1000M;
                Number(form, "motionTurnValue").Value = 100M;
                Call(form, "ToggleMotionTargets", 1, 0);
                Check(Number(form, "targetSpeed").Value == 1000M, "Maximum forward target");
                Call(form, "ToggleMotionTargets", -1, 0);
                Check(Number(form, "targetSpeed").Value == -1000M, "Maximum backward target");
                Call(form, "ToggleMotionTargets", 0, 1);
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == 100M, "Maximum left target");
                Call(form, "ToggleMotionTargets", 0, -1);
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == -100M, "Maximum right target");
                typeof(PidTunerForm).GetField("pendingMotionFrame",
                    BindingFlags.NonPublic | BindingFlags.Instance).SetValue(form, "old motion");
                Call(form, "StopMotion");
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == 0M &&
                    Field<object>(form, "activeMotionDirection").ToString() == "None", "Stop clears targets and direction");
                Check(Field<string>(form, "pendingMotionFrame") == null, "Stop removes stale queued motion");
                Check(Number(form, "verticalKp").Value == 300M &&
                    Number(form, "velocityKp").Value == 0M, "Motion preserves PID gains");
                Call(form, "MotionClicked", 1, 0);
                Check(Number(form, "targetSpeed").Value == 0M &&
                    Number(form, "targetTurn").Value == 0M, "Disconnected click cannot activate motion");
                Call(form, "SetConnected", true);
                Check(Field<Button>(form, "motionForwardButton").Enabled &&
                    Field<Button>(form, "motionStopButton").Enabled, "Connected controls enabled");
                Call(form, "ToggleMotionTargets", 1, 0);
                typeof(PidTunerForm).GetField("pendingMotionFrame",
                    BindingFlags.NonPublic | BindingFlags.Instance).SetValue(form, "old motion");
                Call(form, "SetConnected", false);
                Check(Field<string>(form, "pendingMotionFrame") == null &&
                    Field<object>(form, "activeMotionDirection").ToString() == "None" &&
                    !Field<Button>(form, "motionForwardButton").Enabled, "Disconnect clears selection and queue");
                Call(form, "StopMotion");
                Number(form, "targetSpeedStep").Value = 1M;
                Number(form, "targetTurnStep").Value = 1M;
                Number(form, "verticalKp").Value = 380M;
                Number(form, "verticalKd").Value = 0.396M;
                Number(form, "velocityKp").Value = 0.13M;
                Number(form, "velocityKi").Value = 0.0044M;
                Number(form, "pitchMid").Value = 2M;
                Number(form, "targetSpeed").Value = -7M;
                Number(form, "turnKd").Value = -0.2M;
                Number(form, "targetTurn").Value = 3M;
                Thread.CurrentThread.CurrentCulture = CultureInfo.GetCultureInfo("fr-FR");
                string frame = (string)typeof(PidTunerForm).GetMethod("BuildPidFrame",
                    BindingFlags.NonPublic | BindingFlags.Instance).Invoke(form, null);
                Check(frame == "PID,380,0.396,0.13,0.0044,2,-7,20,-0.2,3\n", "Nine-value invariant protocol");
                Number(form, "velocityKp").Value = 100M;
                frame = (string)typeof(PidTunerForm).GetMethod("BuildPidFrame",
                    BindingFlags.NonPublic | BindingFlags.Instance).Invoke(form, null);
                Check(frame == "PID,380,0.396,100,0.0044,2,-7,20,-0.2,3\n", "Send velocity Kp maximum");
                Check(File.ReadAllText(settings).Contains("Target_Speed_Step=1"), "Saved speed step");
                Number(form, "motionSpeedValue").Value = 20M;
                Number(form, "motionTurnValue").Value = 7M;
                Field<CheckBox>(form, "motionForwardReverse").Checked = true;
                Field<CheckBox>(form, "motionTurnReverse").Checked = true;
            }
            File.AppendAllText(settings, "Target_Speed=999\r\nTarget_Turn=77\r\n");
            using (var restored = new PidTunerForm())
            {
                Check(Number(restored, "verticalKp").Value == 380M, "Preserve saved PID");
                Check(Number(restored, "velocityKp").Value == 100M, "Restore saved velocity Kp maximum");
                Check(Number(restored, "targetSpeed").Value == 0M, "Restart target speed at zero");
                Check(Number(restored, "targetSpeed").Increment == 1M, "Restore speed step");
                Check(Number(restored, "turnKd").Value == -0.2M, "Preserve saved turn gain");
                Check(Number(restored, "targetTurn").Value == 0M, "Restart turn command at zero");
                Check(Field<object>(restored, "activeMotionDirection").ToString() == "None", "Restart with no selected direction");
                Check(Number(restored, "motionSpeedValue").Value == 20M &&
                    Number(restored, "motionTurnValue").Value == 7M, "Restore fixed motion values");
                Check(Field<CheckBox>(restored, "motionForwardReverse").Checked &&
                    Field<CheckBox>(restored, "motionTurnReverse").Checked, "Restore direction inversion");
            }
            Console.WriteLine("PASS: " + checks + " tuner checks");
        }
    }
}
