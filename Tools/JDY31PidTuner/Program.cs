using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Management;
using Microsoft.Win32;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Windows.Forms;

namespace JDY31PidTuner
{
    internal static class Program
    {
        [STAThread]
        private static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            Application.Run(new PidTunerForm());
        }
    }

    internal sealed class PidTunerForm : Form
    {
        private readonly SerialPort serial = new SerialPort();
        private readonly ComboBox portList = new ComboBox();
        private readonly Button connectButton = new Button();
        private readonly Button sendButton = new Button();
        private readonly Label statusLabel = new Label();
        private readonly Label portHintLabel = new Label();
        private readonly TextBox logBox = new TextBox();
        private readonly List<PortInfo> portEntries = new List<PortInfo>();
        private readonly StringBuilder receiveBuffer = new StringBuilder();
        private readonly object receiveBufferLock = new object();
        private readonly object serialIoLock = new object();
        private readonly object responseLock = new object();
        private readonly AutoResetEvent responseEvent = new AutoResetEvent(false);
        private readonly System.Windows.Forms.Timer heartbeatTimer =
            new System.Windows.Forms.Timer();
        private string pendingResponse;
        private string pendingExpectedResponse;
        private int refreshInProgress;
        private int connectInProgress;
        private int requestInProgress;
        private int heartbeatMisses;
        private int pidConfirmationState;
        private volatile bool deviceConnected;
        private volatile bool closing;
        private readonly string settingsPath = Path.Combine(
            AppDomain.CurrentDomain.BaseDirectory, "pid_settings.ini");

        private sealed class PortInfo
        {
            public string PortName { get; private set; }
            public string DisplayName { get; private set; }
            public bool IsBluetooth { get; private set; }
            public bool IsJdy31 { get; private set; }
            public bool IsIncoming { get; private set; }

            public PortInfo(string portName, string displayName, bool isBluetooth,
                bool isJdy31, bool isIncoming)
            {
                PortName = portName;
                DisplayName = displayName;
                IsBluetooth = isBluetooth;
                IsJdy31 = isJdy31;
                IsIncoming = isIncoming;
            }

            public override string ToString()
            {
                return DisplayName;
            }
        }

        private sealed class PnpInfo
        {
            public string Name;
            public string Caption;
            public string Description;
            public string PnpDeviceId;
            public string Manufacturer;
        }

        private readonly System.Windows.Forms.Timer motionSendTimer =
            new System.Windows.Forms.Timer();
        private readonly Button motionForwardButton = new Button();
        private readonly Button motionBackwardButton = new Button();
        private readonly Button motionLeftButton = new Button();
        private readonly Button motionRightButton = new Button();
        private readonly Button motionStopButton = new Button();
        private readonly CheckBox motionForwardReverse = new CheckBox();
        private readonly CheckBox motionTurnReverse = new CheckBox();
        private readonly Label motionTargetLabel = new Label();
        private NumericUpDown motionSpeedValue;
        private NumericUpDown motionTurnValue;
        private string pendingMotionFrame;
        private enum MotionDirection { None, Forward, Backward, Left, Right }
        private MotionDirection activeMotionDirection;
        private bool updatingMotionTargets;

        private NumericUpDown verticalKp;
        private NumericUpDown verticalKd;
        private NumericUpDown velocityKp;
        private NumericUpDown velocityKi;
        private NumericUpDown pitchMid;
        private NumericUpDown targetSpeed;
        private NumericUpDown turnKp;
        private NumericUpDown turnKd;
        private NumericUpDown targetTurn;
        private NumericUpDown verticalKpStep;
        private NumericUpDown verticalKdStep;
        private NumericUpDown velocityKpStep;
        private NumericUpDown velocityKiStep;
        private NumericUpDown pitchMidStep;
        private NumericUpDown targetSpeedStep;
        private NumericUpDown turnKpStep;
        private NumericUpDown turnKdStep;
        private NumericUpDown targetTurnStep;

        public PidTunerForm()
        {
            Text = "JDY-31 \u5e73\u8861\u8f66 PID \u8c03\u53c2\u5668\uff08\u542b\u8fd0\u52a8\u63a7\u5236\uff09";
            StartPosition = FormStartPosition.CenterScreen;
            MinimumSize = new Size(720, 660);
            Size = new Size(900, 740);
            Font = new Font("Microsoft YaHei UI", 10F);

            BuildInterface();
            SetConnected(false);
            LoadPidSettings();
            AttachPidAutoSave();
            SavePidSettings();

            serial.BaudRate = 9600;
            serial.DataBits = 8;
            serial.StopBits = StopBits.One;
            serial.Parity = Parity.None;
            serial.Handshake = Handshake.None;
            serial.ReadTimeout = 1000;
            serial.WriteTimeout = 1000;
            serial.Encoding = System.Text.Encoding.ASCII;
            serial.DataReceived += SerialDataReceived;
            heartbeatTimer.Interval = 10000;
            heartbeatTimer.Tick += HeartbeatTick;
            motionSendTimer.Interval = 50;
            motionSendTimer.Tick += delegate { TrySendPendingMotion(); };
            motionSendTimer.Start();
            FormClosing += FormIsClosing;
            Shown += delegate { RefreshPorts(); };
        }

        protected override void OnLoad(EventArgs e)
        {
            base.OnLoad(e);
            Rectangle area = Screen.FromControl(this).WorkingArea;
            MinimumSize = new Size(Math.Min(MinimumSize.Width, area.Width),
                Math.Min(MinimumSize.Height, area.Height));
            Size = new Size(Math.Min(Width, area.Width), Math.Min(Height, area.Height));
            Location = new Point(area.Left + (area.Width - Width) / 2,
                area.Top + (area.Height - Height) / 2);
        }

        private void BuildInterface()
        {
            var root = new TableLayoutPanel();
            root.Dock = DockStyle.Fill;
            root.Padding = new Padding(16);
            root.ColumnCount = 1;
            root.RowCount = 6;
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100F));
            root.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            Controls.Add(root);

            var connection = new FlowLayoutPanel();
            connection.Dock = DockStyle.Fill;
            connection.AutoSize = true;
            connection.WrapContents = false;
            connection.Padding = new Padding(0, 0, 0, 8);
            connection.Controls.Add(new Label
            {
                Text = "\u84dd\u7259/\u4e32\u53e3:",
                AutoSize = true,
                Margin = new Padding(0, 8, 6, 0)
            });

            portList.DropDownStyle = ComboBoxStyle.DropDownList;
            portList.Width = 520;
            portList.DropDownWidth = 1100;
            portList.Margin = new Padding(0, 4, 8, 0);
            portList.SelectedIndexChanged += delegate { UpdatePortHint(); };
            connection.Controls.Add(portList);

            var refreshButton = new Button
            {
                Text = "\u5237\u65b0\u4e32\u53e3",
                AutoSize = true,
                Margin = new Padding(0, 4, 8, 0)
            };
            refreshButton.Click += delegate { RefreshPorts(); };
            connection.Controls.Add(refreshButton);

            connectButton.Text = "\u8fde\u63a5";
            connectButton.AutoSize = true;
            connectButton.Margin = new Padding(0, 4, 8, 0);
            connectButton.Click += ConnectClicked;
            connection.Controls.Add(connectButton);

            statusLabel.Text = "\u672a\u8fde\u63a5";
            statusLabel.ForeColor = Color.Firebrick;
            statusLabel.AutoSize = true;
            statusLabel.Margin = new Padding(8, 8, 0, 0);
            connection.Controls.Add(statusLabel);
            root.Controls.Add(connection, 0, 0);

            portHintLabel.AutoSize = true;
            portHintLabel.ForeColor = Color.DarkGreen;
            portHintLabel.Padding = new Padding(0, 0, 0, 10);
            root.Controls.Add(portHintLabel, 0, 1);

            var parameterTabs = new TabControl { Dock = DockStyle.Top };
            var balancePage = new TabPage("\u76f4\u7acb\u4e0e\u901f\u5ea6") { AutoScroll = true };
            var steeringPage = new TabPage("\u8f6c\u5411\u73af") { AutoScroll = true };
            parameterTabs.TabPages.Add(balancePage);
            parameterTabs.TabPages.Add(steeringPage);
            var motionPage = new TabPage("\u8fd0\u52a8\u63a7\u5236") { AutoScroll = true };
            parameterTabs.TabPages.Add(motionPage);
            root.Controls.Add(parameterTabs, 0, 2);
            var parameterTable = CreateParameterTable(8);
            balancePage.Controls.Add(parameterTable);

            verticalKp = AddParameterRow(parameterTable, 1,
                "\u76f4\u7acb\u73af Kp (Vertical_Kp)", 300M, -1000M, 1000M, 2, 10M,
                out verticalKpStep);
            verticalKd = AddParameterRow(parameterTable, 2,
                "\u76f4\u7acb\u73af Kd (Vertical_Kd)", 0M, -10M, 10M, 3, 0.01M,
                out verticalKdStep);
            velocityKp = AddParameterRow(parameterTable, 3,
                "\u901f\u5ea6\u73af Kp (Velocity_Kp)", 0M, -20M, 100M, 4, 0.01M,
                out velocityKpStep);
            velocityKi = AddParameterRow(parameterTable, 4,
                "\u901f\u5ea6\u73af Ki (Velocity_Ki)", 0M, -1M, 1M, 4, 0.001M,
                out velocityKiStep);
            pitchMid = AddParameterRow(parameterTable, 5,
                "Pitch \u4e2d\u503c (Med_Angle)", -0.5M, -45M, 45M, 3, 0.1M,
                out pitchMidStep);
            targetSpeed = AddParameterRow(parameterTable, 6,
                "\u76ee\u6807\u901f\u5ea6 (Target_Speed)", 0M, -1000M, 1000M, 0, 1M,
                out targetSpeedStep);
            targetTurn = AddParameterRow(parameterTable, 7,
                "\u8f6c\u5411\u6307\u4ee4 (Target_Turn)", 0M, -100M, 100M, 0, 1M, out targetTurnStep);
            var steeringTable = CreateParameterTable(3);
            steeringPage.Controls.Add(steeringTable);
            turnKp = AddParameterRow(steeringTable, 1,
                "\u8f6c\u5411 Kp (Turn_Kp)", 20M, -200M, 200M, 2, 1M, out turnKpStep);
            turnKd = AddParameterRow(steeringTable, 2,
                "\u8f6c\u5411 Kd (Turn_Kd)", 0M, -5M, 5M, 3, 0.01M, out turnKdStep);
            var steeringHint = new Label { AutoSize = true, ForeColor = Color.DimGray,
                Text = "\u5148\u4fdd\u6301 Kd=0 \u786e\u8ba4\u8f6c\u5411\u65b9\u5411\uff0c\u518d\u786e\u8ba4\u9640\u87ba\u4eea\u963b\u5c3c\u4e3a\u8d1f\u53cd\u9988\u3002" };
            steeringHint.Location = new Point(4, steeringTable.PreferredSize.Height + 12);
            steeringPage.Controls.Add(steeringHint);
            var motionLayout = BuildMotionControlPage(motionPage);
            parameterTabs.Height = Math.Max(parameterTable.PreferredSize.Height,
                motionLayout.PreferredSize.Height) + 42;

            sendButton.Text = "\u53d1\u9001\u5168\u90e8\u53c2\u6570";
            sendButton.Height = 48;
            sendButton.Dock = DockStyle.Top;
            sendButton.Margin = new Padding(0, 16, 0, 12);
            sendButton.BackColor = Color.FromArgb(44, 120, 210);
            sendButton.ForeColor = Color.White;
            sendButton.FlatStyle = FlatStyle.Flat;
            sendButton.Click += SendClicked;
            root.Controls.Add(sendButton, 0, 3);

            logBox.Dock = DockStyle.Fill;
            logBox.Multiline = true;
            logBox.ScrollBars = ScrollBars.Vertical;
            logBox.ReadOnly = true;
            logBox.BackColor = Color.White;
            root.Controls.Add(logBox, 0, 4);

            var hint = new Label();
            hint.AutoSize = true;
            hint.ForeColor = Color.DimGray;
            hint.Padding = new Padding(0, 8, 0, 0);
            hint.Text = "\u8fd0\u52a8\u76ee\u6807\u6bcf\u6b21\u542f\u52a8\u5f52\u96f6\uff1b\u53c2\u6570\u9875\u9700\u70b9\u201c\u53d1\u9001\u5168\u90e8\u53c2\u6570\u201d\uff0c\u8fd0\u52a8\u63a7\u5236\u9875\u65b9\u5411\u6309\u94ae\u76f4\u63a5\u53d1\u9001\u3002";
            root.Controls.Add(hint, 0, 5);
        }

        private TableLayoutPanel BuildMotionControlPage(TabPage page)
        {
            var layout = new TableLayoutPanel
            {
                AutoSize = true, AutoSizeMode = AutoSizeMode.GrowAndShrink,
                ColumnCount = 1, RowCount = 5, Location = new Point(4, 4)
            };
            for (int row = 0; row < 5; row++)
                layout.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            page.Controls.Add(layout);

            var steps = new FlowLayoutPanel { AutoSize = true, WrapContents = false };
            steps.Controls.Add(new Label
            {
                Text = "\u56fa\u5b9a\u901f\u5ea6", AutoSize = true, Margin = new Padding(4, 7, 6, 0)
            });
            motionSpeedValue = CreateMotionValue(targetSpeed.Maximum);
            steps.Controls.Add(motionSpeedValue);
            steps.Controls.Add(new Label
            {
                Text = "\u56fa\u5b9a\u8f6c\u5411\u503c", AutoSize = true, Margin = new Padding(20, 7, 6, 0)
            });
            motionTurnValue = CreateMotionValue(targetTurn.Maximum);
            steps.Controls.Add(motionTurnValue);
            layout.Controls.Add(steps, 0, 0);

            var directions = new FlowLayoutPanel { AutoSize = true, WrapContents = false };
            motionForwardReverse.Text = "\u524d\u540e\u53cd\u5411";
            motionForwardReverse.AutoSize = true;
            motionTurnReverse.Text = "\u5de6\u53f3\u53cd\u5411";
            motionTurnReverse.AutoSize = true;
            motionTurnReverse.Margin = new Padding(20, 3, 3, 3);
            directions.Controls.Add(motionForwardReverse);
            directions.Controls.Add(motionTurnReverse);
            layout.Controls.Add(directions, 0, 1);

            motionTargetLabel.AutoSize = true;
            motionTargetLabel.Font = new Font(Font, FontStyle.Bold);
            motionTargetLabel.Margin = new Padding(4, 6, 4, 8);
            layout.Controls.Add(motionTargetLabel, 0, 2);
            targetSpeed.ValueChanged += MotionTargetChanged;
            targetTurn.ValueChanged += MotionTargetChanged;
            UpdateMotionReadout();

            var pad = new TableLayoutPanel { ColumnCount = 3, RowCount = 3,
                Size = new Size(450, 150), Margin = new Padding(0, 0, 0, 6) };
            for (int i = 0; i < 3; i++)
            {
                pad.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 33.333F));
                pad.RowStyles.Add(new RowStyle(SizeType.Percent, 33.333F));
            }
            AddMotionButton(pad, motionForwardButton, "\u2191 \u524d\u8fdb", 1, 0,
                delegate { MotionClicked(1, 0); });
            AddMotionButton(pad, motionBackwardButton, "\u2193 \u540e\u9000", 1, 2,
                delegate { MotionClicked(-1, 0); });
            AddMotionButton(pad, motionLeftButton, "\u2190 \u5de6\u62d0", 0, 1,
                delegate { MotionClicked(0, 1); });
            AddMotionButton(pad, motionRightButton, "\u53f3\u62d0 \u2192", 2, 1,
                delegate { MotionClicked(0, -1); });
            AddMotionButton(pad, motionStopButton, "\u505c\u6b62\n\u901f\u5ea6/\u8f6c\u5411\u5f52\u96f6", 1, 1,
                delegate { StopMotion(); });
            motionStopButton.BackColor = Color.IndianRed;
            motionStopButton.ForeColor = Color.White;
            layout.Controls.Add(pad, 0, 3);
            layout.Controls.Add(new Label
            {
                Text = "\u540c\u65b9\u5411\u518d\u70b9\u505c\u6b62\uff0c\u5207\u6362\u65b9\u5411\u65f6\u6e05\u9664\u4e0a\u4e00\u6307\u4ee4\uff1b\u6309\u94ae\u7acb\u5373\u53d1\u9001\u5f53\u524d\u5168\u90e8\u53c2\u6570\u3002\n\u505c\u6b62\u5f52\u96f6\u540e\u4fdd\u6301\u5e73\u8861\uff1b\u65b9\u5411\u76f8\u53cd\u65f6\u52fe\u9009\u5bf9\u5e94\u53cd\u5411\u9009\u9879\u3002",
                AutoSize = true, MaximumSize = new Size(640, 0),
                ForeColor = Color.DimGray, Margin = new Padding(4, 4, 4, 4)
            }, 0, 4);
            return layout;
        }

        private static NumericUpDown CreateMotionValue(decimal maximum)
        {
            var control = CreateNumberBox(0M, 0M, maximum, 0);
            control.Dock = DockStyle.None;
            control.Width = 100;
            control.Increment = 1M;
            return control;
        }

        private static void AddMotionButton(TableLayoutPanel pad, Button button,
            string text, int column, int row, EventHandler clicked)
        {
            button.Text = text;
            button.Dock = DockStyle.Fill;
            button.Margin = new Padding(4);
            button.Click += clicked;
            pad.Controls.Add(button, column, row);
        }

        private void UpdateMotionReadout()
        {
            motionTargetLabel.Text = string.Format(CultureInfo.InvariantCulture,
                "\u76ee\u6807\u901f\u5ea6\uff1a{0}    \u8f6c\u5411\u6307\u4ee4\uff1a{1}", targetSpeed.Value, targetTurn.Value);
        }

        private void MotionTargetChanged(object sender, EventArgs e)
        {
            if (!updatingMotionTargets)
            {
                activeMotionDirection = MotionDirection.None;
                UpdateMotionButtonState();
            }
            UpdateMotionReadout();
        }

        private void UpdateMotionButtonState()
        {
            SetMotionButtonState(motionForwardButton, MotionDirection.Forward);
            SetMotionButtonState(motionBackwardButton, MotionDirection.Backward);
            SetMotionButtonState(motionLeftButton, MotionDirection.Left);
            SetMotionButtonState(motionRightButton, MotionDirection.Right);
        }

        private void SetMotionButtonState(Button button, MotionDirection direction)
        {
            bool selected = activeMotionDirection == direction;
            button.BackColor = selected ? Color.FromArgb(44, 120, 210) : SystemColors.Control;
            button.ForeColor = selected ? Color.White : SystemColors.ControlText;
            button.UseVisualStyleBackColor = !selected;
        }

        private void ToggleMotionTargets(int speedDirection, int turnDirection)
        {
            MotionDirection requested = speedDirection > 0 ? MotionDirection.Forward :
                speedDirection < 0 ? MotionDirection.Backward :
                turnDirection > 0 ? MotionDirection.Left :
                turnDirection < 0 ? MotionDirection.Right : MotionDirection.None;
            SetMotionCommand(requested == activeMotionDirection ? MotionDirection.None : requested);
        }

        private void SetMotionCommand(MotionDirection direction)
        {
            decimal speedSign = motionForwardReverse.Checked ? -1M : 1M;
            decimal turnSign = motionTurnReverse.Checked ? -1M : 1M;
            updatingMotionTargets = true;
            try
            {
                activeMotionDirection = direction;
                targetSpeed.Value = direction == MotionDirection.Forward ? speedSign * motionSpeedValue.Value :
                    direction == MotionDirection.Backward ? -speedSign * motionSpeedValue.Value : 0M;
                targetTurn.Value = direction == MotionDirection.Left ? turnSign * motionTurnValue.Value :
                    direction == MotionDirection.Right ? -turnSign * motionTurnValue.Value : 0M;
                if (targetSpeed.Value == 0M && targetTurn.Value == 0M)
                    activeMotionDirection = MotionDirection.None;
            }
            finally
            {
                updatingMotionTargets = false;
            }
            UpdateMotionButtonState();
            UpdateMotionReadout();
        }

        private void MotionClicked(int speedDirection, int turnDirection)
        {
            if (!deviceConnected || !serial.IsOpen)
                return;
            ToggleMotionTargets(speedDirection, turnDirection);
            QueueMotionCommand();
        }

        private void StopMotion()
        {
            pendingMotionFrame = null;
            SetMotionCommand(MotionDirection.None);
            QueueMotionCommand();
        }

        private void QueueMotionCommand()
        {
            if (!deviceConnected || !serial.IsOpen)
                return;
            // Replace queued motion with the newest targets; never replay old clicks.
            pendingMotionFrame = BuildPidFrame();
            TrySendPendingMotion();
        }

        private void TrySendPendingMotion()
        {
            if (pendingMotionFrame == null)
                return;
            if (closing || !deviceConnected || !serial.IsOpen)
            {
                pendingMotionFrame = null;
                return;
            }
            if (Interlocked.CompareExchange(ref requestInProgress, 0, 0) != 0)
                return;
            string frame = pendingMotionFrame;
            pendingMotionFrame = null;
            SendPidFrame(frame);
        }

        private static TableLayoutPanel CreateParameterTable(int rows)
        {
            var table = new TableLayoutPanel { Anchor = AnchorStyles.Top | AnchorStyles.Left,
                AutoSize = true, CellBorderStyle = TableLayoutPanelCellBorderStyle.Single,
                ColumnCount = 5, RowCount = rows, Location = new Point(4, 4) };
            table.RowStyles.Add(new RowStyle(SizeType.Absolute, 38F));
            for (int row = 1; row < rows; row++)
                table.RowStyles.Add(new RowStyle(SizeType.AutoSize));
            foreach (float width in new[] { 190F, 135F, 40F, 130F, 55F })
                table.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, width));
            AddHeader(table, "\u53c2\u6570", 0);
            AddHeader(table, "\u5f53\u524d\u503c\uff08\u53ef\u7528\u7bad\u5934\uff09", 1);
            AddHeader(table, "\u6b65\u8fdb", 2);
            AddHeader(table, "\u6b65\u8fdb\u8bbe\u7f6e", 3);
            AddHeader(table, "\u5f52\u96f6", 4);
            return table;
        }

        private static void AddHeader(TableLayoutPanel table, string text, int column)
        {
            var label = new Label();
            label.Text = text;
            label.Dock = DockStyle.Fill;
            label.TextAlign = ContentAlignment.MiddleCenter;
            label.Font = new Font(label.Font, FontStyle.Bold);
            label.Padding = new Padding(4, 8, 4, 8);
            table.Controls.Add(label, column, 0);
        }

        private static NumericUpDown AddParameterRow(TableLayoutPanel table, int row,
            string name, decimal initial, decimal minimum, decimal maximum,
            int decimalPlaces, decimal initialStep, out NumericUpDown step)
        {
            var nameLabel = new Label();
            nameLabel.Text = name;
            nameLabel.Dock = DockStyle.Fill;
            nameLabel.TextAlign = ContentAlignment.MiddleLeft;
            nameLabel.Padding = new Padding(8, 5, 4, 5);
            table.Controls.Add(nameLabel, 0, row);

            NumericUpDown value = CreateNumberBox(initial, minimum, maximum, decimalPlaces);
            table.Controls.Add(value, 1, row);

            var stepLabel = new Label();
            stepLabel.Text = "\u2195";
            stepLabel.Dock = DockStyle.Fill;
            stepLabel.TextAlign = ContentAlignment.MiddleCenter;
            table.Controls.Add(stepLabel, 2, row);

            bool integerParameter = decimalPlaces == 0;
            NumericUpDown stepControl = CreateNumberBox(initialStep,
                integerParameter ? 1M : 0.0001M, 100M, integerParameter ? 0 : 4);
            stepControl.Increment = integerParameter ? 1M : 0.0001M;
            value.Increment = stepControl.Value;
            stepControl.ValueChanged += delegate { value.Increment = stepControl.Value; };
            table.Controls.Add(stepControl, 3, row);
            step = stepControl;

            var resetButton = new Button();
            resetButton.Text = "\u5f52\u96f6";
            resetButton.Dock = DockStyle.Fill;
            resetButton.Margin = new Padding(5);
            resetButton.Click += delegate { value.Value = 0M; };
            table.Controls.Add(resetButton, 4, row);
            return value;
        }

        private static NumericUpDown CreateNumberBox(decimal value, decimal minimum,
            decimal maximum, int decimalPlaces)
        {
            var box = new NumericUpDown();
            box.Dock = DockStyle.Fill;
            box.Margin = new Padding(8, 6, 8, 6);
            box.Minimum = minimum;
            box.Maximum = maximum;
            box.DecimalPlaces = decimalPlaces;
            box.Value = value;
            box.TextAlign = HorizontalAlignment.Right;
            box.ThousandsSeparator = false;
            return box;
        }

        private void AttachPidAutoSave()
        {
            verticalKp.ValueChanged += ParameterValueChanged;
            verticalKd.ValueChanged += ParameterValueChanged;
            velocityKp.ValueChanged += ParameterValueChanged;
            velocityKi.ValueChanged += ParameterValueChanged;
            pitchMid.ValueChanged += ParameterValueChanged;
            targetSpeed.ValueChanged += ParameterValueChanged;
            turnKp.ValueChanged += ParameterValueChanged;
            turnKd.ValueChanged += ParameterValueChanged;
            targetTurn.ValueChanged += ParameterValueChanged;
            verticalKpStep.ValueChanged += ParameterValueChanged;
            verticalKdStep.ValueChanged += ParameterValueChanged;
            velocityKpStep.ValueChanged += ParameterValueChanged;
            velocityKiStep.ValueChanged += ParameterValueChanged;
            pitchMidStep.ValueChanged += ParameterValueChanged;
            targetSpeedStep.ValueChanged += ParameterValueChanged;
            turnKpStep.ValueChanged += ParameterValueChanged;
            turnKdStep.ValueChanged += ParameterValueChanged;
            targetTurnStep.ValueChanged += ParameterValueChanged;
            motionSpeedValue.ValueChanged += ParameterValueChanged;
            motionTurnValue.ValueChanged += ParameterValueChanged;
            motionForwardReverse.CheckedChanged += ParameterValueChanged;
            motionTurnReverse.CheckedChanged += ParameterValueChanged;
        }

        private void ParameterValueChanged(object sender, EventArgs e)
        {
            SavePidSettings();
        }

        private void LoadPidSettings()
        {
            if (!File.Exists(settingsPath))
                return;

            try
            {
                var values = new Dictionary<string, decimal>(
                    StringComparer.OrdinalIgnoreCase);
                foreach (string rawLine in File.ReadAllLines(settingsPath))
                {
                    string line = rawLine.Trim();
                    if ((line.Length == 0) || line.StartsWith("#"))
                        continue;

                    int separator = line.IndexOf('=');
                    decimal value;
                    if ((separator <= 0) ||
                        !decimal.TryParse(line.Substring(separator + 1).Trim(),
                            NumberStyles.Float, CultureInfo.InvariantCulture, out value))
                        continue;

                    values[line.Substring(0, separator).Trim()] = value;
                }

                ApplySavedValue(values, "Vertical_Kp", verticalKp);
                ApplySavedValue(values, "Vertical_Kd", verticalKd);
                ApplySavedValue(values, "Velocity_Kp", velocityKp);
                ApplySavedValue(values, "Velocity_Ki", velocityKi);
                ApplySavedValue(values, "Med_Angle", pitchMid);
                ApplySavedValue(values, "Turn_Kp", turnKp);
                ApplySavedValue(values, "Turn_Kd", turnKd);
                ApplySavedValue(values, "Vertical_Kp_Step", verticalKpStep);
                ApplySavedValue(values, "Vertical_Kd_Step", verticalKdStep);
                ApplySavedValue(values, "Velocity_Kp_Step", velocityKpStep);
                ApplySavedValue(values, "Velocity_Ki_Step", velocityKiStep);
                ApplySavedValue(values, "Med_Angle_Step", pitchMidStep);
                ApplySavedValue(values, "Target_Speed_Step", targetSpeedStep);
                ApplySavedValue(values, "Turn_Kp_Step", turnKpStep);
                ApplySavedValue(values, "Turn_Kd_Step", turnKdStep);
                ApplySavedValue(values, "Target_Turn_Step", targetTurnStep);
                ApplySavedValue(values, "Motion_Speed", motionSpeedValue);
                ApplySavedValue(values, "Motion_Turn", motionTurnValue);
                decimal direction;
                if (values.TryGetValue("Motion_ForwardReverse", out direction))
                    motionForwardReverse.Checked = direction == 1M;
                if (values.TryGetValue("Motion_TurnReverse", out direction))
                    motionTurnReverse.Checked = direction == 1M;
                // Speed/steering commands start at zero rather than restoring motion.
                AppendLog("已加载上次保存的 PID 参数");
            }
            catch (Exception ex)
            {
                AppendLog("参数文件读取失败: " + ex.Message);
            }
        }

        private static void ApplySavedValue(Dictionary<string, decimal> values,
            string name, NumericUpDown control)
        {
            decimal value;
            if (values.TryGetValue(name, out value) &&
                (value >= control.Minimum) && (value <= control.Maximum) &&
                (control.DecimalPlaces != 0 || decimal.Truncate(value) == value))
                control.Value = value;
        }

        private void SavePidSettings()
        {
            try
            {
                string content =
                    "# JDY-31 PID tuner auto-saved parameters\r\n" +
                    "Vertical_Kp=" + verticalKp.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Vertical_Kd=" + verticalKd.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Velocity_Kp=" + velocityKp.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Velocity_Ki=" + velocityKi.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Med_Angle=" + pitchMid.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Turn_Kp=" + turnKp.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Turn_Kd=" + turnKd.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Vertical_Kp_Step=" + verticalKpStep.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Vertical_Kd_Step=" + verticalKdStep.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Velocity_Kp_Step=" + velocityKpStep.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Velocity_Ki_Step=" + velocityKiStep.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Med_Angle_Step=" + pitchMidStep.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Target_Speed_Step=" + targetSpeedStep.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Turn_Kp_Step=" + turnKpStep.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Turn_Kd_Step=" + turnKdStep.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Target_Turn_Step=" + targetTurnStep.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Motion_Speed=" + motionSpeedValue.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Motion_Turn=" + motionTurnValue.Value.ToString(CultureInfo.InvariantCulture) + "\r\n" +
                    "Motion_ForwardReverse=" + (motionForwardReverse.Checked ? "1" : "0") + "\r\n" +
                    "Motion_TurnReverse=" + (motionTurnReverse.Checked ? "1" : "0") + "\r\n";
                File.WriteAllText(settingsPath, content, new UTF8Encoding(false));
            }
            catch (Exception ex)
            {
                AppendLog("参数自动保存失败: " + ex.Message);
            }
        }

        private void RefreshPorts()
        {
            if (Interlocked.Exchange(ref refreshInProgress, 1) != 0)
                return;

            string previous = GetSelectedPortName();
            ThreadPool.QueueUserWorkItem(delegate
            {
                List<PortInfo> ports = null;
                Exception error = null;
                try
                {
                    ports = LoadPortEntries();
                }
                catch (Exception ex)
                {
                    error = ex;
                }

                Interlocked.Exchange(ref refreshInProgress, 0);
                RunOnUi(delegate
                {
                    if (error != null)
                    {
                        AppendLog("\u5237\u65b0\u4e32\u53e3\u5931\u8d25: " + error.Message);
                        return;
                    }
                    ApplyPortEntries(previous, ports);
                });
            });
        }

        private void ApplyPortEntries(string previous, List<PortInfo> ports)
        {
            portEntries.Clear();
            portEntries.AddRange(ports);
            portList.Items.Clear();
            foreach (PortInfo port in ports)
                portList.Items.Add(port);

            int selectedIndex = ports.FindIndex(delegate(PortInfo port)
            {
                return string.Equals(port.PortName, previous,
                    StringComparison.OrdinalIgnoreCase) && !port.IsIncoming;
            });
            if (selectedIndex >= 0)
                portList.SelectedIndex = selectedIndex;
            else if (ports.Count > 0)
            {
                int jdy31Index = ports.FindIndex(delegate(PortInfo port)
                {
                    return port.IsJdy31 && !port.IsIncoming;
                });
                portList.SelectedIndex = jdy31Index >= 0 ? jdy31Index : 0;
            }

            UpdatePortHint();
            AppendLog("\u627e\u5230\u4e32\u53e3: " +
                (ports.Count == 0 ? "\u65e0" :
                string.Join("; ", ports.Select(port => port.DisplayName).ToArray())));
        }

        private void RunOnUi(MethodInvoker action)
        {
            if (closing || IsDisposed || !IsHandleCreated)
                return;
            try
            {
                BeginInvoke(action);
            }
            catch (ObjectDisposedException)
            {
                // The form may be closing while a serial/WMI worker completes.
            }
            catch (InvalidOperationException)
            {
                // The form may be closing while a serial/WMI worker completes.
            }
        }

        private void UpdatePortHint()
        {
            PortInfo selected = portList.SelectedItem as PortInfo;
            if (selected == null)
            {
                portHintLabel.Text = "\u672a\u627e\u5230\u4e32\u53e3";
                portHintLabel.ForeColor = Color.Firebrick;
            }
            else if (selected.IsJdy31)
            {
                if (selected.IsIncoming)
                {
                    portHintLabel.Text = selected.PortName +
                        " = JDY-31 \u84dd\u7259\u4f20\u5165\u7aef\u53e3\uff08\u4e0d\u80fd\u7528\u5b83\u4e3b\u52a8\u8fde\u63a5\uff09";
                    portHintLabel.ForeColor = Color.Firebrick;
                }
                else
                {
                    portHintLabel.Text = selected.PortName +
                        " = JDY-31-SPP \u4f20\u51fa\u7aef\u53e3\uff08\u63a8\u8350\uff09";
                    portHintLabel.ForeColor = Color.DarkGreen;
                }
            }
            else if (selected.IsBluetooth)
            {
                portHintLabel.Text = selected.PortName +
                    " = \u5176\u4ed6\u84dd\u7259\u865a\u62df\u4e32\u53e3\uff08\u672a\u5339\u914d JDY-31\uff09";
                portHintLabel.ForeColor = Color.DarkOrange;
            }
            else
            {
                portHintLabel.Text = selected.PortName +
                    " = \u666e\u901a\u4e32\u53e3\uff08\u4e0d\u662f JDY-31 \u84dd\u7259\uff09";
                portHintLabel.ForeColor = Color.DarkOrange;
            }
        }

        private string GetSelectedPortName()
        {
            PortInfo selected = portList.SelectedItem as PortInfo;
            return selected == null ? null : selected.PortName;
        }

        private static List<PortInfo> LoadPortEntries()
        {
            var ports = new Dictionary<string, PortInfo>(StringComparer.OrdinalIgnoreCase);

            try
            {
                using (var searcher = new ManagementObjectSearcher(
                    "SELECT DeviceID, Name, Description, PNPDeviceID, Manufacturer FROM Win32_SerialPort"))
                using (ManagementObjectCollection devices = searcher.Get())
                {
                    foreach (ManagementObject device in devices)
                    {
                        AddPortEntry(ports,
                            ReadWmiProperty(device, "DeviceID"),
                            ReadWmiProperty(device, "Name"),
                            ReadWmiProperty(device, "Description"),
                            ReadWmiProperty(device, "PNPDeviceID"),
                            ReadWmiProperty(device, "Manufacturer"), "");
                    }
                }
            }
            catch
            {
                // Use PnP and SerialPort APIs below when this WMI provider is unavailable.
            }

            LoadPnPPortEntries(ports);
            LoadBluetoothRegistryPortEntries(ports);

            foreach (string port in SerialPort.GetPortNames())
                AddPortEntry(ports, port, "", "", "", "", "");

            return ports.Values.OrderBy(port => port.IsIncoming)
                .ThenBy(port => GetPortNumber(port.PortName))
                .ThenBy(port => port.PortName, StringComparer.OrdinalIgnoreCase).ToList();
        }

        private static int GetPortNumber(string portName)
        {
            int number;
            return portName.Length > 3 &&
                int.TryParse(portName.Substring(3), out number) ? number : int.MaxValue;
        }

        private static void LoadPnPPortEntries(Dictionary<string, PortInfo> ports)
        {
            var devices = new List<PnpInfo>();
            try
            {
                using (var searcher = new ManagementObjectSearcher(
                    "SELECT Name, Caption, Description, PNPDeviceID, Manufacturer FROM Win32_PnPEntity"))
                using (ManagementObjectCollection results = searcher.Get())
                {
                    foreach (ManagementObject device in results)
                    {
                        devices.Add(new PnpInfo
                        {
                            Name = ReadWmiProperty(device, "Name"),
                            Caption = ReadWmiProperty(device, "Caption"),
                            Description = ReadWmiProperty(device, "Description"),
                            PnpDeviceId = ReadWmiProperty(device, "PNPDeviceID"),
                            Manufacturer = ReadWmiProperty(device, "Manufacturer")
                        });
                    }
                }
            }
            catch
            {
                return;
            }

            var bluetoothNames = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            foreach (PnpInfo device in devices)
            {
                if (!ContainsBluetoothMarker(device.PnpDeviceId))
                    continue;

                string address = ExtractBluetoothAddress(device.PnpDeviceId);
                string deviceName = FirstNonEmpty(device.Name, device.Caption);
                if (!string.IsNullOrWhiteSpace(address) &&
                    !string.IsNullOrWhiteSpace(deviceName) &&
                    ExtractPortName(deviceName) == "")
                    bluetoothNames[address] = deviceName;
            }

            foreach (PnpInfo device in devices)
            {
                string portName = ExtractPortName(FirstNonEmpty(device.Name, device.Caption));
                if (portName == "")
                    continue;

                string bluetoothName = "";
                string address = ExtractBluetoothAddress(device.PnpDeviceId);
                if (!string.IsNullOrWhiteSpace(address))
                    bluetoothNames.TryGetValue(address, out bluetoothName);

                AddPortEntry(ports, portName, device.Name, device.Description,
                    device.PnpDeviceId, device.Manufacturer, bluetoothName);
            }
        }

        private static string ReadWmiProperty(ManagementObject device, string property)
        {
            object value = device[property];
            return value == null ? "" : value.ToString();
        }

        private static void LoadBluetoothRegistryPortEntries(
            Dictionary<string, PortInfo> ports)
        {
            const string registryPath = @"SYSTEM\CurrentControlSet\Enum\BTHENUM";
            var devices = new List<PnpInfo>();

            try
            {
                using (RegistryKey bluetoothRoot =
                    Registry.LocalMachine.OpenSubKey(registryPath))
                {
                    if (bluetoothRoot == null)
                        return;

                    foreach (string serviceName in bluetoothRoot.GetSubKeyNames())
                    using (RegistryKey serviceKey = bluetoothRoot.OpenSubKey(serviceName))
                    {
                        if (serviceKey == null)
                            continue;

                        foreach (string instanceName in serviceKey.GetSubKeyNames())
                        using (RegistryKey instanceKey = serviceKey.OpenSubKey(instanceName))
                        {
                            if (instanceKey == null)
                                continue;

                            string friendlyName = Convert.ToString(
                                instanceKey.GetValue("FriendlyName", ""));
                            string portName = "";
                            using (RegistryKey parameters =
                                instanceKey.OpenSubKey("Device Parameters"))
                            {
                                if (parameters != null)
                                    portName = Convert.ToString(
                                        parameters.GetValue("PortName", ""));
                            }

                            devices.Add(new PnpInfo
                            {
                                Name = friendlyName,
                                Caption = friendlyName,
                                Description = portName,
                                PnpDeviceId = "BTHENUM\\" + serviceName +
                                    "\\" + instanceName,
                                Manufacturer = "Microsoft Bluetooth"
                            });
                        }
                    }
                }
            }
            catch
            {
                return;
            }

            var bluetoothNames = new Dictionary<string, string>(
                StringComparer.OrdinalIgnoreCase);
            foreach (PnpInfo device in devices)
            {
                string address = ExtractBluetoothAddress(device.PnpDeviceId);
                if (!string.IsNullOrWhiteSpace(address) &&
                    !string.IsNullOrWhiteSpace(device.Name) &&
                    string.IsNullOrWhiteSpace(device.Description))
                    bluetoothNames[address] = device.Name;
            }

            foreach (PnpInfo device in devices)
            {
                string portName = device.Description;
                if (string.IsNullOrWhiteSpace(portName) ||
                    !portName.StartsWith("COM", StringComparison.OrdinalIgnoreCase))
                    continue;

                string bluetoothName = "";
                string address = ExtractBluetoothAddress(device.PnpDeviceId);
                if (!string.IsNullOrWhiteSpace(address))
                    bluetoothNames.TryGetValue(address, out bluetoothName);

                AddPortEntry(ports, portName, device.Name, "Bluetooth SPP",
                    device.PnpDeviceId, device.Manufacturer, bluetoothName);
            }
        }

        private static void AddPortEntry(Dictionary<string, PortInfo> ports,
            string portName, string name, string description, string pnpDeviceId,
            string manufacturer, string bluetoothDeviceName)
        {
            if (string.IsNullOrWhiteSpace(portName) ||
                !portName.StartsWith("COM", StringComparison.OrdinalIgnoreCase))
                return;

            bool isBluetooth = ContainsBluetoothMarker(name) ||
                ContainsBluetoothMarker(description) ||
                ContainsBluetoothMarker(pnpDeviceId) ||
                !string.IsNullOrWhiteSpace(bluetoothDeviceName);
            bool isJdy31 = ContainsJdy31Marker(name) ||
                ContainsJdy31Marker(description) ||
                ContainsJdy31Marker(pnpDeviceId) ||
                ContainsJdy31Marker(bluetoothDeviceName);
            bool isIncoming = isBluetooth && IsIncomingBluetoothPort(name,
                description, pnpDeviceId);

            string detail = string.IsNullOrWhiteSpace(name) ?
                "No device description" : name;
            if (!string.IsNullOrWhiteSpace(bluetoothDeviceName) &&
                detail.IndexOf(bluetoothDeviceName, StringComparison.OrdinalIgnoreCase) < 0)
                detail = bluetoothDeviceName + " | " + detail;
            if (!string.IsNullOrWhiteSpace(description) &&
                !string.Equals(description, name, StringComparison.OrdinalIgnoreCase))
                detail += " | " + description;
            if (!string.IsNullOrWhiteSpace(manufacturer) &&
                detail.IndexOf(manufacturer, StringComparison.OrdinalIgnoreCase) < 0)
                detail += " | " + manufacturer;
            if (!string.IsNullOrWhiteSpace(pnpDeviceId))
                detail += " | ID:" + pnpDeviceId;

            string category = isJdy31 && isIncoming ?
                "[JDY-31 \u4f20\u5165\u7aef\u53e3\uff08\u4e0d\u53ef\u7528\uff09] " :
                isJdy31 ? "[JDY-31 \u4f20\u51fa\u7aef\u53e3\uff08\u8bf7\u9009\u62e9\uff09] " :
                isBluetooth && isIncoming ? "[\u84dd\u7259\u4f20\u5165\u7aef\u53e3\uff08\u4e0d\u53ef\u7528\uff09] " :
                isBluetooth ? "[\u84dd\u7259\u4e32\u53e3] " : "[\u666e\u901a\u4e32\u53e3] ";
            string display = portName.ToUpperInvariant() + " | " + category + detail;
            PortInfo candidate = new PortInfo(portName.ToUpperInvariant(), display,
                isBluetooth, isJdy31, isIncoming);
            PortInfo existing;
            if (!ports.TryGetValue(candidate.PortName, out existing) ||
                (candidate.IsJdy31 && !existing.IsJdy31) ||
                (candidate.IsBluetooth && !candidate.IsIncoming && existing.IsIncoming) ||
                (candidate.IsBluetooth && !existing.IsBluetooth) ||
                (existing.DisplayName.IndexOf("No device description",
                    StringComparison.OrdinalIgnoreCase) >= 0 &&
                 candidate.DisplayName.IndexOf("No device description",
                    StringComparison.OrdinalIgnoreCase) < 0))
                ports[candidate.PortName] = candidate;
        }

        private static bool ContainsBluetoothMarker(string text)
        {
            return !string.IsNullOrWhiteSpace(text) &&
                (text.IndexOf("BTHENUM", StringComparison.OrdinalIgnoreCase) >= 0 ||
                 text.IndexOf("BLUETOOTH", StringComparison.OrdinalIgnoreCase) >= 0 ||
                 text.IndexOf("\u84dd\u7259", StringComparison.OrdinalIgnoreCase) >= 0 ||
                 text.IndexOf("JDY-31", StringComparison.OrdinalIgnoreCase) >= 0 ||
                 text.IndexOf("HC-05", StringComparison.OrdinalIgnoreCase) >= 0 ||
                 text.IndexOf("Serial over Bluetooth", StringComparison.OrdinalIgnoreCase) >= 0);
        }

        private static bool ContainsJdy31Marker(string text)
        {
            return !string.IsNullOrWhiteSpace(text) &&
                text.IndexOf("JDY-31", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private static bool IsIncomingBluetoothPort(params string[] values)
        {
            foreach (string value in values)
            {
                if (string.IsNullOrWhiteSpace(value))
                    continue;
                if (value.IndexOf("_LOCALMFG&0000", StringComparison.OrdinalIgnoreCase) >= 0 ||
                    value.IndexOf("000000000000_", StringComparison.OrdinalIgnoreCase) >= 0 ||
                    value.IndexOf("INCOMING", StringComparison.OrdinalIgnoreCase) >= 0 ||
                    value.IndexOf("\u4f20\u5165", StringComparison.OrdinalIgnoreCase) >= 0 ||
                    value.IndexOf("\u50b3\u5165", StringComparison.OrdinalIgnoreCase) >= 0)
                    return true;
            }
            return false;
        }

        private static string FirstNonEmpty(params string[] values)
        {
            foreach (string value in values)
            {
                if (!string.IsNullOrWhiteSpace(value))
                    return value;
            }
            return "";
        }

        private static string ExtractPortName(string text)
        {
            if (string.IsNullOrWhiteSpace(text))
                return "";

            Match match = Regex.Match(text, @"\((COM\d+)\)", RegexOptions.IgnoreCase);
            if (!match.Success)
                match = Regex.Match(text, @"\b(COM\d+)\b", RegexOptions.IgnoreCase);
            return match.Success ? match.Groups[1].Value.ToUpperInvariant() : "";
        }

        private static string ExtractBluetoothAddress(string text)
        {
            if (string.IsNullOrWhiteSpace(text))
                return "";

            Match match = Regex.Match(text, @"DEV_([0-9A-F]{12})", RegexOptions.IgnoreCase);
            if (!match.Success)
                match = Regex.Match(text, @"&([0-9A-F]{12})(?:_|$)", RegexOptions.IgnoreCase);
            return match.Success ? match.Groups[1].Value.ToUpperInvariant() : "";
        }

        private void ConnectClicked(object sender, EventArgs e)
        {
            if (Interlocked.CompareExchange(ref connectInProgress, 1, 0) != 0)
                return;

            if (serial.IsOpen)
            {
                Interlocked.Exchange(ref connectInProgress, 0);
                SetConnected(false);
                CloseSerial();
                AppendLog("\u5df2\u65ad\u5f00");
                return;
            }

            PortInfo selected = portList.SelectedItem as PortInfo;
            if (selected == null)
            {
                Interlocked.Exchange(ref connectInProgress, 0);
                MessageBox.Show("\u8bf7\u5148\u9009\u62e9\u4e32\u53e3", "\u6ca1\u6709\u4e32\u53e3",
                    MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            if (selected.IsIncoming)
            {
                Interlocked.Exchange(ref connectInProgress, 0);
                MessageBox.Show("\u8fd9\u662f\u84dd\u7259\u201c\u4f20\u5165\u201dCOM\u7aef\u53e3\uff0c\u65e0\u6cd5\u4e3b\u52a8\u8fde\u63a5 JDY-31\u3002\r\n" +
                    "\u8bf7\u9009\u62e9\u6807\u6709 OUTGOING/\u4f20\u51fa\u7684 JDY-31 \u7aef\u53e3\u3002",
                    "\u4e32\u53e3\u65b9\u5411\u4e0d\u5bf9", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }

            connectButton.Enabled = false;
            portList.Enabled = false;
            statusLabel.Text = "\u6b63\u5728\u8fde\u63a5...";
            statusLabel.ForeColor = Color.DarkOrange;
            AppendLog("CONNECTING " + selected.DisplayName);

            ThreadPool.QueueUserWorkItem(delegate
            {
                Exception error = null;
                int attempts = selected.IsBluetooth ? 2 : 1;
                for (int attempt = 1; attempt <= attempts; attempt++)
                {
                    try
                    {
                        lock (serialIoLock)
                        {
                            if (closing)
                                return;
                            if (serial.IsOpen)
                                serial.Close();
                            serial.PortName = selected.PortName;
                            serial.Open();
                            serial.DiscardInBuffer();
                            serial.DiscardOutBuffer();
                        }
                        lock (receiveBufferLock)
                            receiveBuffer.Clear();

                        string handshakeResponse;
                        if (!SendProtocolRequest("PING\n", 3000, "PONG",
                                out handshakeResponse) ||
                            !string.Equals(handshakeResponse, "PONG",
                                StringComparison.OrdinalIgnoreCase))
                            throw new TimeoutException(
                                "\u4e32\u53e3\u5df2\u6253\u5f00\uff0c\u4f46\u4e0b\u4f4d\u673a\u672a\u54cd\u5e94 PING\u63e1\u624b\u3002");
                        error = null;
                        break;
                    }
                    catch (Exception ex)
                    {
                        error = ex;
                        lock (serialIoLock)
                        {
                            if (serial.IsOpen)
                                serial.Close();
                        }
                        if ((attempt >= attempts) || !IsSemaphoreTimeout(ex))
                            break;
                        RunOnUi(delegate
                        {
                            AppendLog("Windows \u84dd\u7259\u4e32\u53e3\u9996\u6b21\u6253\u5f00\u8d85\u65f6\uff0c\u6b63\u5728\u91cd\u8bd5...");
                        });
                        Thread.Sleep(700);
                    }
                }

                Interlocked.Exchange(ref connectInProgress, 0);
                if (closing)
                    return;
                RunOnUi(delegate
                {
                    if (error == null)
                    {
                        SetConnected(true);
                        AppendLog("CONNECTED \u4e0b\u4f4d\u673a\u63e1\u624b\u6210\u529f " +
                            selected.DisplayName + " @ 9600 8N1");
                        if (!selected.IsJdy31)
                            AppendLog("WARNING: \u5f53\u524d\u4e32\u53e3\u4e0d\u662f\u5df2\u8bc6\u522b\u7684 JDY-31-SPP");
                    }
                    else
                    {
                        SetConnected(false);
                        string message = FormatConnectError(error);
                        AppendLog("CONNECT ERROR: " + message.Replace("\r", " ").Replace("\n", " "));
                        MessageBox.Show(message, "\u8fde\u63a5\u5931\u8d25", MessageBoxButtons.OK,
                            MessageBoxIcon.Error);
                    }
                });
            });
        }

        private static bool IsSemaphoreTimeout(Exception error)
        {
            return (error.HResult & 0xFFFF) == 121 ||
                error.Message.IndexOf("semaphore", StringComparison.OrdinalIgnoreCase) >= 0 ||
                error.Message.IndexOf("\u4fe1\u53f7\u706f\u8d85\u65f6", StringComparison.OrdinalIgnoreCase) >= 0 ||
                error.Message.IndexOf("\u4fe1\u865f\u71c8\u903e\u6642", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        private static string FormatConnectError(Exception error)
        {
            if (IsSemaphoreTimeout(error))
                return "Windows \u84dd\u7259\u865a\u62df\u4e32\u53e3\u8fde\u63a5\u8d85\u65f6\uff08\u9519\u8bef 121\uff09\u3002\r\n\r\n" +
                    "\u8bf7\u786e\u8ba4\uff1a\r\n" +
                    "1. JDY-31 \u5df2\u4e0a\u7535\uff0c\u4e14\u6ca1\u6709\u88ab\u624b\u673a\u6216\u5176\u4ed6\u8bbe\u5907\u8fde\u63a5\uff1b\r\n" +
                    "2. \u9009\u62e9\u7684\u662f OUTGOING/\u4f20\u51fa COM \u7aef\u53e3\uff0c\u4e0d\u662f INCOMING/\u4f20\u5165\u7aef\u53e3\uff1b\r\n" +
                    "3. \u5fc5\u8981\u65f6\u91cd\u65b0\u7ed9 JDY-31 \u4e0a\u7535\u5e76\u91cd\u65b0\u914d\u5bf9\u3002";
            if (error is UnauthorizedAccessException)
                return "\u4e32\u53e3\u88ab\u5176\u4ed6\u7a0b\u5e8f\u5360\u7528\u6216\u5f53\u524d\u7528\u6237\u65e0\u6743\u6253\u5f00\u3002\r\n" +
                    "\u8bf7\u5173\u95ed\u5176\u4ed6\u4e32\u53e3\u52a9\u624b\u540e\u91cd\u8bd5\u3002";
            return error.Message;
        }

        private void SendClicked(object sender, EventArgs e)
        {
            pendingMotionFrame = null;
            SendPidFrame(BuildPidFrame());
        }

        private void SendPidFrame(string frame)
        {
            if (!deviceConnected || !serial.IsOpen)
            {
                MessageBox.Show("\u8bf7\u5148\u8fde\u63a5 JDY-31 \u84dd\u7259\u4e32\u53e3", "\u5c1a\u672a\u8fde\u63a5",
                    MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }
            if (Interlocked.CompareExchange(ref requestInProgress, 1, 0) != 0)
            {
                MessageBox.Show("\u6b63\u5728\u7b49\u5f85\u4e0b\u4f4d\u673a\u54cd\u5e94\uff0c\u8bf7\u7a0d\u540e\u518d\u8bd5\u3002",
                    "\u901a\u4fe1\u5fd9", MessageBoxButtons.OK, MessageBoxIcon.Information);
                return;
            }

            sendButton.Enabled = false;
            ThreadPool.QueueUserWorkItem(delegate
            {
                Exception error = null;
                string response = null;
                Interlocked.Exchange(ref pidConfirmationState, 1);
                try
                {
                    RunOnUi(delegate { AppendLog("TX  " + frame.Trim()); });
                    if (!SendProtocolRequest(frame, 8000, "OK", out response))
                        throw new TimeoutException("\u53d1\u9001\u540e 8 \u79d2\u5185\u6682\u672a\u6536\u5230 PID \u786e\u8ba4\uff0c\u7ee7\u7eed\u7b49\u5f85\u8fdf\u5230\u5e94\u7b54\u3002");
                }
                catch (Exception ex)
                {
                    error = ex;
                }
                RunOnUi(delegate
                {
                    if (error != null || !string.Equals(response, "OK", StringComparison.OrdinalIgnoreCase))
                        pendingMotionFrame = null;
                    Interlocked.Exchange(ref requestInProgress, 0);
                    sendButton.Enabled = deviceConnected;
                    if (error == null)
                    {
                        Interlocked.Exchange(ref pidConfirmationState, 0);
                        if (string.Equals(response, "OK", StringComparison.OrdinalIgnoreCase))
                        {
                            Interlocked.Exchange(ref heartbeatMisses, 0);
                            AppendLog("SUCCESS \u4e0b\u4f4d\u673a\u5df2\u786e\u8ba4\u5e94\u7528 PID \u53c2\u6570");
                        }
                        else
                            AppendLog("REJECTED \u4e0b\u4f4d\u673a\u8fd4\u56de: " + response);
                        return;
                    }

                    if (error is TimeoutException)
                    {
                        Interlocked.CompareExchange(ref pidConfirmationState, 2, 1);
                        AppendLog("WAITING: " + error.Message);
                    }
                    else
                    {
                        AppendLog("TX ERROR: " + error.Message);
                        SetConnected(false);
                        CloseSerial();
                        MessageBox.Show(error.Message, "\u84dd\u7259\u901a\u4fe1\u9519\u8bef",
                            MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                });
            });
        }

        private string BuildPidFrame()
        {
            return string.Format(CultureInfo.InvariantCulture,
                "PID,{0:0.####},{1:0.####},{2:0.####},{3:0.####},{4:0.####},{5:0},{6:0.####},{7:0.####},{8:0}\n",
                verticalKp.Value, verticalKd.Value, velocityKp.Value,
                velocityKi.Value, pitchMid.Value, targetSpeed.Value,
                turnKp.Value, turnKd.Value, targetTurn.Value);
        }

        private bool SendProtocolRequest(string frame, int timeoutMilliseconds,
            string expectedResponse, out string response)
        {
            responseEvent.Reset();
            lock (responseLock)
            {
                pendingResponse = null;
                pendingExpectedResponse = expectedResponse;
            }

            lock (serialIoLock)
            {
                if (!serial.IsOpen)
                    throw new InvalidOperationException("\u4e32\u53e3\u5df2\u65ad\u5f00");
                serial.Write(frame);
            }

            bool signaled = responseEvent.WaitOne(timeoutMilliseconds);
            lock (responseLock)
            {
                response = pendingResponse;
                pendingResponse = null;
                pendingExpectedResponse = null;
            }
            return signaled && response != null;
        }

        private void CaptureProtocolResponse(string line)
        {
            bool matched = false;
            if (string.Equals(line, "OK", StringComparison.OrdinalIgnoreCase) &&
                Interlocked.Exchange(ref pidConfirmationState, 0) == 2)
            {
                RunOnUi(delegate
                {
                    AppendLog("LATE SUCCESS \u5df2\u6536\u5230\u8fdf\u5230 OK\uff0c\u4e0b\u4f4d\u673a\u5df2\u5e94\u7528 PID \u53c2\u6570");
                });
            }
            lock (responseLock)
            {
                if (pendingExpectedResponse != null &&
                    (string.Equals(line, pendingExpectedResponse,
                        StringComparison.OrdinalIgnoreCase) ||
                     line.StartsWith("ERR,", StringComparison.OrdinalIgnoreCase)))
                {
                    pendingResponse = line;
                    matched = true;
                }
            }
            if (matched)
                responseEvent.Set();
        }

        private void HeartbeatTick(object sender, EventArgs e)
        {
            if (closing || !deviceConnected || !serial.IsOpen ||
                Interlocked.CompareExchange(ref requestInProgress, 1, 0) != 0)
                return;

            ThreadPool.QueueUserWorkItem(delegate
            {
                bool alive = false;
                try
                {
                    string response;
                    alive = SendProtocolRequest("PING\n", 8000, "PONG", out response) &&
                        string.Equals(response, "PONG", StringComparison.OrdinalIgnoreCase);
                }
                catch
                {
                    alive = false;
                }
                Interlocked.Exchange(ref requestInProgress, 0);

                if (alive)
                {
                    Interlocked.Exchange(ref heartbeatMisses, 0);
                }
                else if (deviceConnected)
                {
                    int misses = Interlocked.Increment(ref heartbeatMisses);
                    RunOnUi(delegate
                    {
                        if (!deviceConnected)
                            return;
                        if (misses < 3)
                        {
                            AppendLog("WARNING \u4e0b\u4f4d\u673a\u5fc3\u8df3\u6682\u65e0\u54cd\u5e94 (" +
                                misses + "/3)\uff0c\u4fdd\u6301\u8fde\u63a5\u7ee7\u7eed\u68c0\u6d4b");
                            return;
                        }
                        AppendLog("DISCONNECTED \u4e0b\u4f4d\u673a\u8fde\u7eed 3 \u6b21\u5fc3\u8df3\u8d85\u65f6");
                        SetConnected(false);
                        CloseSerial();
                    });
                }
            });
        }

        private void SerialDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string text;
                lock (serialIoLock)
                {
                    if (closing || !serial.IsOpen)
                        return;
                    text = serial.ReadExisting();
                }
                var lines = new List<string>();
                lock (receiveBufferLock)
                {
                    receiveBuffer.Append(text);
                    while (true)
                    {
                        string buffered = receiveBuffer.ToString();
                        int newline = buffered.IndexOf('\n');
                        if (newline < 0)
                            break;

                        string line = buffered.Substring(0, newline).TrimEnd('\r');
                        receiveBuffer.Remove(0, newline + 1);
                        if (!string.IsNullOrWhiteSpace(line))
                            lines.Add(line);
                    }
                }

                if (lines.Count == 0)
                    return;

                RunOnUi(delegate
                {
                    foreach (string line in lines)
                    {
                        if (!string.Equals(line, "PONG", StringComparison.OrdinalIgnoreCase))
                            AppendLog("RX  " + line);
                    }
                });

                foreach (string line in lines)
                    CaptureProtocolResponse(line);
            }
            catch
            {
                // The port may have been closed while the callback was pending.
            }
        }

        private void SetConnected(bool connected)
        {
            deviceConnected = connected;
            if (!connected)
            {
                pendingMotionFrame = null;
                activeMotionDirection = MotionDirection.None;
                UpdateMotionButtonState();
            }
            motionForwardButton.Enabled = connected;
            motionBackwardButton.Enabled = connected;
            motionLeftButton.Enabled = connected;
            motionRightButton.Enabled = connected;
            motionStopButton.Enabled = connected;
            Interlocked.Exchange(ref heartbeatMisses, 0);
            if (connected)
                heartbeatTimer.Start();
            else
                heartbeatTimer.Stop();
            connectButton.Enabled = true;
            sendButton.Enabled = connected;
            connectButton.Text = connected ? "\u65ad\u5f00" : "\u8fde\u63a5";
            statusLabel.Text = connected ? "\u5df2\u8fde\u63a5" : "\u672a\u8fde\u63a5";
            statusLabel.ForeColor = connected ? Color.ForestGreen : Color.Firebrick;
            portList.Enabled = !connected;
        }

        private void AppendLog(string text)
        {
            if (string.IsNullOrWhiteSpace(text))
                return;
            logBox.AppendText(DateTime.Now.ToString("HH:mm:ss") + "  " + text +
                Environment.NewLine);
            if (logBox.TextLength > 120000)
            {
                logBox.Select(0, logBox.TextLength - 80000);
                logBox.SelectedText = "";
                logBox.SelectionStart = logBox.TextLength;
            }
        }

        private void CloseSerial()
        {
            lock (serialIoLock)
            {
                if (serial.IsOpen)
                    serial.Close();
            }
            lock (receiveBufferLock)
                receiveBuffer.Clear();
        }

        private void FormIsClosing(object sender, FormClosingEventArgs e)
        {
            SavePidSettings();
            closing = true;
            pendingMotionFrame = null;
            motionSendTimer.Stop();
            motionSendTimer.Dispose();
            heartbeatTimer.Stop();
            heartbeatTimer.Dispose();
            responseEvent.Set();
            lock (serialIoLock)
            {
                if (serial.IsOpen)
                    serial.Close();
                serial.Dispose();
            }
            responseEvent.Dispose();
        }
    }
}
