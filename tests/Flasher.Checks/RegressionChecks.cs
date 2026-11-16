using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Security.Cryptography;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Threading;

namespace WpfApp1;

internal static class RegressionChecks
{
    private static int count;
    private static void Check(bool passed, string label)
    {
        if (!passed) throw new InvalidOperationException(label);
        Console.WriteLine($"PASS {++count}: {label}");
    }

    [STAThread]
    public static int Main(string[] args)
    {
        if (args.FirstOrDefault() == "--chip")
        {
            string port = args[Array.IndexOf(args, "--port") + 1];
            Console.WriteLine("MAC: AA:BB:CC:DD:EE:01");
            Console.Error.WriteLine($"write {port} {Path.GetFileName(args[^1])}");
            return port == Environment.GetEnvironmentVariable("FLASHER_CHECK_FAIL") ? 2 : 0;
        }

        var repo = new DirectoryInfo(AppContext.BaseDirectory);
        while (repo != null && !File.Exists(Path.Combine(repo.FullName, "Flasher.sln"))) repo = repo.Parent;
        if (repo == null) throw new DirectoryNotFoundException("Run the checks inside the Flasher repository.");
        var temp = Directory.CreateTempSubdirectory("Flasher-check-");
        try
        {
            var root = temp.FullName;
            foreach (var folder in new[] { "projects", "projectsard", "partsheme 1.0.4.rc1" })
            {
                foreach (var file in Directory.GetFiles(Path.Combine(repo.FullName, folder), "*", SearchOption.AllDirectories)
                    .Where(f => f.EndsWith(".bin") || Path.GetFileName(f) == "ino.json"))
                {
                    string target = Path.Combine(root, Path.GetRelativePath(repo.FullName, file));
                    Directory.CreateDirectory(Path.GetDirectoryName(target));
                    File.Copy(file, target);
                }
            }
            var toolDir = Directory.CreateDirectory(Path.Combine(root, "partsheme")).FullName;
            File.Copy(Path.Combine(repo.FullName, "partsheme", "boot_app0.bin"), Path.Combine(toolDir, "boot_app0.bin"));
            foreach (var file in Directory.GetFiles(AppContext.BaseDirectory).Where(p => p.EndsWith(".dll") || p.EndsWith(".json")))
                File.Copy(file, Path.Combine(toolDir, Path.GetFileName(file)));
            File.Copy(Path.Combine(AppContext.BaseDirectory, "Flasher.Checks.exe"), Path.Combine(toolDir, "esptool.exe"));

            SynchronizationContext.SetSynchronizationContext(new DispatcherSynchronizationContext());
            var app = new App();
            app.InitializeComponent();
            app.ShutdownMode = ShutdownMode.OnExplicitShutdown;
            Properties.Settings.Default.Reset();
            CheckQueue(root);
            CheckPresets(root);
            Console.WriteLine($"Passed {count} checks.");
            return 0;
        }
        finally
        {
            if (temp.Parent?.FullName.Equals(Path.TrimEndingDirectorySeparator(Path.GetFullPath(Path.GetTempPath())), StringComparison.OrdinalIgnoreCase) == true &&
                temp.Name.StartsWith("Flasher-check-", StringComparison.Ordinal)) temp.Delete(true);
        }
    }

    private static void Open(Window window)
    {
        window.WindowStartupLocation = WindowStartupLocation.Manual;
        window.Left = -8000;
        window.ShowInTaskbar = false;
        window.ShowActivated = false;
        window.Show();
        window.UpdateLayout();
        Pump(window);
    }

    private static void Pump(Window w) => w.Dispatcher.Invoke(() => { }, DispatcherPriority.ApplicationIdle);
    private static void Press(Button button) => button.RaiseEvent(new RoutedEventArgs(Button.ClickEvent));
    private static IEnumerable<Button> Buttons(DependencyObject container)
    {
        for (int i = 0; i < VisualTreeHelper.GetChildrenCount(container); i++)
        {
            var child = VisualTreeHelper.GetChild(container, i);
            if (child is Button b) yield return b;
            foreach (var button in Buttons(child)) yield return button;
        }
    }

    private static void Write(MainWindow w, Button button)
    {
        if (!button.IsEnabled) throw new InvalidOperationException("Write button disabled");
        Press(button);
        var timeout = DateTime.UtcNow.AddSeconds(20);
        while (!w.queuePanel.IsEnabled && DateTime.UtcNow < timeout) { Pump(w); Thread.Sleep(8); }
        Pump(w);
        if (!w.queuePanel.IsEnabled) throw new TimeoutException("esptool simulation did not finish");
    }

    private static void CheckQueue(string root)
    {
        string home = Path.Combine(root, "operator");
        var w = new MainWindow(root, home);
        Open(w);
        var rows = w.lb.Items.Cast<FlashInfo>().ToArray();
        var first = rows[0];
        var second = rows[1];
        var profiles = first.DirectoryProject;
        Check(rows.Length == 5 && profiles.Count == 10 && profiles.All(p => p.ValidationError == null), "all 10 shipped kits pass preflight");
        foreach (var profile in profiles)
        {
            using var package = profile.Prepare(root, "COM91", false);
            var arguments = package.Command.ArgumentList;
            Check(arguments.Contains("--erase-all") == (profile.NameDir is "testfenster" or "testteplica") &&
                new[] { "0x1000", "0x8000", "0xe000", "0x10000" }.All(arguments.Contains), "addresses and erase policy for " + profile.NameDir);
        }

        first.Path = "testfenster"; first.Port = "COM91";
        second.Path = "testteplica"; second.Port = "COM91";
        w.chk_all.RaiseEvent(new RoutedEventArgs(CheckBox.ClickEvent));
        Check(first.chk && second.chk && rows.Skip(2).All(r => !r.chk), "select all skips empty rows");
        Check(!w.flash_all.IsEnabled && w.selectionHint.Text.Contains("COM91"), "duplicate port blocks batch");
        w.chk_all.RaiseEvent(new RoutedEventArgs(CheckBox.ClickEvent));
        Check(rows.All(r => !r.chk), "second select-all click clears the queue selection");
        rows[4].chk = true;
        w.chk_all.RaiseEvent(new RoutedEventArgs(CheckBox.ClickEvent));
        Check(first.chk && second.chk && !rows[4].chk, "select all removes a previously checked incomplete row");
        second.Port = "COM92"; w.chk_erase_Copy.IsChecked = true; w.chk_noClose.IsChecked = false;
        var profile2 = second.SelectedProfile;
        var files = profile2.CreateFlashCommand(root, "COM92", false).ArgumentList.TakeLast(8).Chunk(2).Select(pair => pair[1]).ToArray();
        foreach (var file in files)
        {
            var original = File.ReadAllBytes(file);
            try
            {
                File.WriteAllBytes(file, new byte[] { 0xe9 });
                try { using var kit = profile2.Prepare(root, "COM92", false); throw new Exception("one-byte file accepted"); }
                catch (InvalidDataException) { Check(true, "truncated file rejected: " + Path.GetFileName(file)); }
                File.WriteAllBytes(file, original);
                var changed = (byte[])original.Clone();
                changed[changed.Length / 2] ^= 1;
                File.WriteAllBytes(file, changed);
                try { using var kit = profile2.Prepare(root, "COM92", false); throw new Exception("corrupt file accepted"); }
                catch (InvalidDataException) { Check(true, "corruption rejected: " + Path.GetFileName(file)); }
            }
            finally { File.WriteAllBytes(file, original); }
        }
        void RejectChange(string path, Action<byte[]> change, string label)
        {
            var original = File.ReadAllBytes(path);
            try
            {
                var changed = (byte[])original.Clone();
                change(changed);
                File.WriteAllBytes(path, changed);
                try { using var kit = profile2.Prepare(root, "COM92", true); throw new Exception("accepted: " + label); }
                catch (InvalidDataException) { Check(true, label); }
            }
            finally { File.WriteAllBytes(path, original); }
        }
        static void EditTable(byte[] table, int entry, int field, uint value)
        {
            BinaryPrimitives.WriteUInt32LittleEndian(table.AsSpan(entry * 32 + field), value);
            int marker = Enumerable.Range(0, table.Length / 32).First(i => table[i * 32] == 0xeb) * 32;
            MD5.HashData(table.AsSpan(0, marker)).CopyTo(table, marker + 16);
        }
        RejectChange(files[0], image => image[12] = 5, "wrong target chip rejected");
        RejectChange(files[3], image => image[^1] ^= 1, "tampered SHA-256 rejected with intact image checksum");
        RejectChange(files[3], image => BinaryPrimitives.WriteUInt32LittleEndian(image.AsSpan(28), uint.MaxValue), "oversized segment rejected");
        RejectChange(files[1], table => table[12] ^= 1, "partition MD5 rejected");
        RejectChange(files[1], table => EditTable(table, 1, 4, 0x9000), "overlapping partitions rejected with a valid MD5");
        RejectChange(files[1], table => EditTable(table, 2, 8, 0x10000), "app exceeding its partition rejected");
        RejectChange(files[1], table => EditTable(table, 3, 8, 0x400000), "partition outside flash rejected");
        using (var held = profile2.Prepare(root, "COM92", true))
        {
            try { using var writer = File.OpenWrite(files[^1]); throw new Exception("image could change after validation"); }
            catch (IOException) { Check(true, "verified images locked through write"); }
        }

        var appPath = files[^1];
        File.Move(appPath, appPath + ".hold");
        Write(w, w.flash_all);
        Check(!second.Ready && !w.flash_all.IsEnabled && !first.HasLog && !Directory.Exists(Path.Combine(home, "logs")),
            "missing image in second job prevents starting either job and disables the bad kit");
        File.Move(appPath + ".hold", appPath); Press(w.Update); Pump(w);
        Check(second.Ready && w.flash_all.IsEnabled, "repaired kit enabled on refresh");

        Write(w, w.flash_all);
        Check(first.Stage == FlashStage.Success && second.Stage == FlashStage.Success && SerialPort.Active.Count == 2, "two successful jobs open two monitors");
        void Feed(string port, string data) { SerialPort.Active[port].Feed(data); Pump(w); }
        Feed("COM91", "btn_dlin_off moter 1 nekrutitsa\r\n");
        Check(w.txt_err.Text.Contains("Ожидание проверки"), "pending hardware check is distinct from failure");
        Feed("COM91", "O");
        Check(w.partialLog.Text.Contains("COM91") && w.partialLog.Text.EndsWith("O"), "incomplete serial line is visible before the newline");
        Feed("COM92", "Error: Could not read temperature data\r\n"); Feed("COM91", "K\r\n");
        Check(w.txt_err.Text.Contains("COM91 · testfenster — Успех") && w.txt_err.Text.Contains("COM92 · testteplica — Ошибка"), "split serial lines and simultaneous results isolated by board");
        Feed("COM92", "OK\n");
        Check(!w.txt_err.Text.Contains("Ошибка"), "new success replaces previous error");
        Feed("COM92", "[HTTPS] GET... code: 503\n");
        Check(w.txt_err.Text.Contains("503") && !w.txt_err.Text.Contains("COM92 · testteplica — Успех"), "HTTP failure replaces success");
        var monitor91 = SerialPort.Active["COM91"]; string history = w.txt_log.Text;
        Press(w.Update); Pump(w);
        Check(SerialPort.Active["COM91"] == monitor91 && w.txt_log.Text.Contains(history) && w.txt_err.Text.Contains("503"), "refresh retains connected monitor, log and result");
        SerialPort.Ports = new[] { "COM91" }; Press(w.Update); Pump(w);
        Check(second.Port == "COM92" && !second.Ready && !SerialPort.Active.ContainsKey("COM92") &&
            !w.txt_err.Text.Contains("COM92") && w.txt_err.Text.Contains("COM91") && second.flMAC == null,
            "disconnect closes only the missing port and releases its result and MAC");
        SerialPort.Ports = new[] { "COM91", "COM92" }; Press(w.Update); Pump(w);
        Check(second.Port == "COM92" && second.Ready && second.Stage != FlashStage.Success, "reconnected selection restored without claiming old success");

        Environment.SetEnvironmentVariable("FLASHER_CHECK_FAIL", "COM92");
        Write(w, w.flash_all);
        Check(first.Stage == FlashStage.Success && second.Stage == FlashStage.Failure && second.flMAC == null &&
            SerialPort.Active.Keys.SequenceEqual(new[] { "COM91" }), "failed last job leaves earlier monitor running");
        Environment.SetEnvironmentVariable("FLASHER_CHECK_FAIL", "COM91");
        Write(w, w.flash_all);
        Check(first.Stage == FlashStage.Failure && second.Stage == FlashStage.Success && SerialPort.Active.ContainsKey("COM92"), "later job runs after first fails");
        Environment.SetEnvironmentVariable("FLASHER_CHECK_FAIL", null);
        Write(w, w.flash_all);
        var oldLog = first.LastLog; string contents = File.ReadAllText(oldLog);
        Check(contents.Contains("testfenster") && contents.Contains("COM91") && contents.Contains("AABBCCDDEE01") &&
            contents.Contains("Прошито") && contents.Contains("write COM91"), "attempt log captures profile, port, MAC, stderr and result");

        var third = rows[2]; third.Path = "SmartKitchen2"; third.Port = "COM91";
        var single = Buttons(w.lb).Single(b => b.Content as string == "Прошить" && b.DataContext == third);
        Write(w, single);
        Check(third.CanEditPreset && !first.CanEditPreset && first.Stage == FlashStage.Historical &&
            first.LastLog == oldLog && oldLog != third.LastLog && File.ReadAllText(oldLog) == contents, "port reuse preserves original log and invalidates previous board");
        Feed("COM91", "OK\nsucces\nno wifi\n");
        Check(!w.txt_err.Text.Contains("SmartKitchen2"), "factory firmware cannot report a test result");
        third.Path = "testirovanieFan";
        Write(w, single);
        Feed("COM91", "suc"); Feed("COM91", "ces\nno wifi\nerror\n");
        Check(w.txt_err.Text.Contains("no wifi") && !w.txt_err.Text.Contains("Успех"), "fan failure overrides success with its diagnostic");
        Feed("COM91", "succes\n");
        Check(w.txt_err.Text.Contains("Успех") && !w.txt_err.Text.Contains("no wifi") && !third.CanEditPreset, "fan recovery and preset availability");
        SerialPort.BusyPort = "COM91";
        Write(w, single);
        Check(third.Stage == FlashStage.Success && !SerialPort.Active.ContainsKey("COM91") && w.txt_log.Text.Contains("Порт занят"), "busy monitor does not turn a completed write into a failure");
        SerialPort.BusyPort = null;
        w.chk_erase.IsChecked = true; w.Close();
        var restored = new MainWindow(root, home);
        Open(restored);
        var saved = (FlashInfo)restored.lb.Items[0];
        Check(saved.Path == "testfenster" && saved.Port == "COM91" && saved.chk && restored.chk_erase.IsChecked == true, "queue and write options restored on reopen");
        Check(saved.LastLog == oldLog && saved.HasLog && saved.Stage == FlashStage.Idle, "restored history points to the original attempt without claiming a current board");
        restored.Close();
    }

    private static void CheckPresets(string root)
    {
        var dir = Path.Combine(root, "presets");
        const string mac = "AABBCCDDEE01";
        var settings = new Settings("SmartKitchen2", mac, dir);
        Open(settings);
        var devices = settings.lb_settings.Items.Cast<DeviceIOT>().ToArray();
        settings.lb_settings.UnselectAll();
        Check(devices.Length == 6 && devices[0].numPIN == null, "v2 keeps unknown GPIO unset and deselection is safe");
        settings.lb_settings.ScrollIntoView(devices[5]);
        settings.UpdateLayout();
        devices[5].Type = "svet"; Pump(settings);
        Check(Buttons(settings.lb_settings).Single(b => b.DataContext == devices[5]).IsEnabled, "changing an unused device to a light enables its editor");
        var light = new Svet(devices[0]);
        Open(light); light.txt_numPIN.Text = "13"; Press(light.btn_Save);
        var motor = new Retrotop(devices[4]);
        Open(motor); motor.txt_numPIN2.Text = "14"; motor.txt_numBUTTON2.Text = "34"; Press(motor.btn_Save);
        Check(devices[4].numPIN2 == 14 && devices[4].numButton2 == 34, "second pair editable and accepts input-only button GPIO");
        settings.Close();
        var preset = new LocalPreset("SmartKitchen2", mac, dir);
        Check(preset.Devices[0].numPIN == null && preset.Devices[5].Type == "none", "closing without save discards edits");
        try { preset.Save(); throw new Exception("unset output accepted"); }
        catch (InvalidDataException) { Check(!Directory.Exists(dir), "unset output blocks save"); }
        foreach (int pin in new[] { -1, 6, 20, 24, 28, 31, 111, 999 })
            Check(!DeviceIOT.ValidGpio(pin, false), "invalid GPIO rejected: " + pin);
        Check(!DeviceIOT.ValidGpio(34, true) && DeviceIOT.ValidGpio(34, false), "input-only GPIO rejected as output");
        preset.Devices[0].numPIN = 13; preset.Devices[4].numPIN2 = 14; preset.Save();
        var reopened = new Settings("SmartKitchen2", mac, dir);
        var savedDevices = reopened.lb_settings.Items.Cast<DeviceIOT>().ToArray();
        savedDevices[1].numButton = 35;
        reopened.Loaded += (_, _) => reopened.Dispatcher.InvokeAsync(() => Press(reopened.btn_Post));
        reopened.Left = -8000;
        reopened.WindowStartupLocation = WindowStartupLocation.Manual;
        reopened.ShowInTaskbar = false;
        Check(reopened.ShowDialog() == true && new LocalPreset("SmartKitchen2", mac, dir).Devices[1].numButton == 35, "save button writes the edited preset to disk");
        Check(new LocalPreset("SmartKitchen2", mac, dir).Devices[4].numPIN2 == 14 &&
            new LocalPreset("SmartKitchen2", "AABBCCDDEE02", dir).Devices[0].numPIN == null &&
            new LocalPreset("SmartKitchen", mac, dir).Devices[0].numPIN == 32, "presets survive reopen and are isolated by MAC and model");
    }
}
