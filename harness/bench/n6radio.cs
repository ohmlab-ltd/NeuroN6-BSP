/*
 * n6radio.cs - this PC's radios as bench fixtures for the Neuro N6 (Windows).
 *
 * BLE through the Windows Runtime, and a Wi-Fi Direct group owner in
 * legacy mode, which is an ordinary WPA2 access point the board can join as a
 * station. Nothing is installed and no setting is changed: every fixture lasts
 * only as long as the command runs. Built by build_n6radio.py with the .NET
 * Framework's own C# 5 compiler, so it needs nothing beyond Windows 10 or 11.
 *
 *   n6radio scan      [--seconds N] [--name PREFIX] [--first]
 *   n6radio gatt      ADDRESS [--list] [--read UUID]... [--write UUID=HEX]...
 *                     [--notify UUID] [--seconds N]
 *   n6radio pair      ADDRESS [--pin PIN]
 *   n6radio unpair    ADDRESS
 *   n6radio advertise [--seconds N] [--mfg HEX]
 *   n6radio server    [--seconds N]
 *   n6radio wfd-ap    --ssid SSID --key KEY [--seconds N]
 *
 * One result per line, a keyword first, so a script can parse the output.
 * Exit status 0 means the command did what it was asked.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.Advertisement;
using Windows.Devices.Bluetooth.GenericAttributeProfile;
using Windows.Devices.Enumeration;
using Windows.Devices.WiFiDirect;
using Windows.Foundation;
using Windows.Security.Credentials;
using Windows.Storage.Streams;

static class N6Radio
{
    /* The bench GATT server's service and characteristic, also used by the
     * board's central checks. */
    static readonly Guid BenchService = new Guid("6e360001-b5a3-f393-e0a9-e50e24dcca9e");
    static readonly Guid BenchValue   = new Guid("6e360002-b5a3-f393-e0a9-e50e24dcca9e");

    static int Main(string[] args)
    {
        try
        {
            return Run(args).GetAwaiter().GetResult();
        }
        catch (Exception ex)
        {
            Console.WriteLine("ERROR " + ex.GetType().Name + ": " + ex.Message);
            return 2;
        }
    }

    static async Task<int> Run(string[] args)
    {
        if (args.Length == 0)
        {
            Console.WriteLine("usage: n6radio scan|gatt|pair|unpair|advertise|server|wfd-ap ...");
            return 2;
        }
        List<string> rest = args.Skip(1).ToList();
        switch (args[0])
        {
            case "scan":      return await Scan(rest);
            case "gatt":      return await Gatt(rest);
            case "pair":      return await Pair(rest);
            case "unpair":    return await Unpair(rest);
            case "advertise": return await Advertise(rest);
            case "server":    return await Server(rest);
            case "wfd-ap":    return await WfdAp(rest);
            case "adapter":   return await Adapter();
        }
        Console.WriteLine("ERROR unknown command " + args[0]);
        return 2;
    }

    /* Windows Runtime plumbing, written out: System.Runtime.WindowsRuntime.dll's
     * helpers need the Windows SDK's union metadata to compile against. */

    static Task<T> W<T>(IAsyncOperation<T> op)
    {
        TaskCompletionSource<T> tcs = new TaskCompletionSource<T>();
        op.Completed = (info, status) =>
        {
            if (status == AsyncStatus.Completed) tcs.TrySetResult(info.GetResults());
            else if (status == AsyncStatus.Error) tcs.TrySetException(info.ErrorCode);
            else tcs.TrySetCanceled();
        };
        return tcs.Task;
    }

    static byte[] Bytes(IBuffer b)
    {
        byte[] r = new byte[b.Length];
        if (b.Length > 0) DataReader.FromBuffer(b).ReadBytes(r);
        return r;
    }

    static IBuffer Buf(byte[] b)
    {
        DataWriter w = new DataWriter();
        w.WriteBytes(b);
        return w.DetachBuffer();
    }

    /* Arguments. */

    static string Opt(List<string> a, string name, string def)
    {
        int i = a.IndexOf(name);
        return (i >= 0 && i + 1 < a.Count) ? a[i + 1] : def;
    }

    static List<string> Opts(List<string> a, string name)
    {
        List<string> r = new List<string>();
        for (int i = 0; i + 1 < a.Count; i++) if (a[i] == name) r.Add(a[i + 1]);
        return r;
    }

    static bool Flag(List<string> a, string name) { return a.Contains(name); }

    static ulong ParseAddress(string s)
    {
        return Convert.ToUInt64(s.Replace(":", "").Replace("-", ""), 16);
    }

    static string FormatAddress(ulong a)
    {
        string h = a.ToString("X12");
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 12; i += 2)
        {
            if (i > 0) sb.Append(':');
            sb.Append(h, i, 2);
        }
        return sb.ToString();
    }

    static Guid ParseUuid(string s)
    {
        if (s.Length <= 4) return BluetoothUuidHelper.FromShortId(Convert.ToUInt32(s, 16));
        return new Guid(s);
    }

    static string Hex(byte[] b)
    {
        return BitConverter.ToString(b).Replace("-", "").ToLowerInvariant();
    }

    static byte[] FromHex(string h)
    {
        byte[] b = new byte[h.Length / 2];
        for (int i = 0; i < b.Length; i++) b[i] = Convert.ToByte(h.Substring(2 * i, 2), 16);
        return b;
    }

    static string Printable(byte[] b)
    {
        StringBuilder sb = new StringBuilder();
        foreach (byte c in b) sb.Append(c >= 32 && c < 127 ? (char)c : '.');
        return sb.ToString();
    }

    /* The adapter. */

    static async Task<int> Adapter()
    {
        BluetoothAdapter a = await W(BluetoothAdapter.GetDefaultAsync());
        if (a == null) { Console.WriteLine("ERROR no BLE adapter"); return 2; }
        Console.WriteLine("ADAPTER address=" + FormatAddress(a.BluetoothAddress) +
                          " le=" + a.IsLowEnergySupported +
                          " central=" + a.IsCentralRoleSupported +
                          " peripheral=" + a.IsPeripheralRoleSupported +
                          " offload=" + a.IsAdvertisementOffloadSupported +
                          " extended=" + a.IsExtendedAdvertisingSupported +
                          " maxadv=" + a.MaxAdvertisementDataLength);
        return 0;
    }

    /* Scan. */

    static async Task<int> Scan(List<string> a)
    {
        int seconds = int.Parse(Opt(a, "--seconds", "8"));
        string prefix = Opt(a, "--name", null);
        bool first = Flag(a, "--first");
        TaskCompletionSource<bool> found = new TaskCompletionSource<bool>();
        HashSet<string> seen = new HashSet<string>();
        object gate = new object();

        BluetoothLEAdvertisementWatcher w = new BluetoothLEAdvertisementWatcher();
        w.ScanningMode = BluetoothLEScanningMode.Active;
        w.Received += (s, e) =>
        {
            string name = e.Advertisement.LocalName ?? "";
            if (prefix != null && !name.StartsWith(prefix, StringComparison.Ordinal)) return;
            StringBuilder mfg = new StringBuilder();
            foreach (BluetoothLEManufacturerData m in e.Advertisement.ManufacturerData)
                mfg.Append(m.CompanyId.ToString("x4") + ":" + Hex(Bytes(m.Data)) + " ");
            string uuids = string.Join(",", e.Advertisement.ServiceUuids.Select(u => u.ToString()));
            string line = "ADV addr=" + FormatAddress(e.BluetoothAddress) +
                          " rssi=" + e.RawSignalStrengthInDBm +
                          " type=" + e.AdvertisementType +
                          " name=\"" + name + "\"" +
                          (uuids.Length > 0 ? " uuids=" + uuids : "") +
                          (mfg.Length > 0 ? " mfg=" + mfg.ToString().Trim() : "");
            lock (gate)
            {
                if (seen.Add(line)) Console.WriteLine(line);
            }
            if (prefix != null) found.TrySetResult(true);
        };
        w.Start();
        Task done = await Task.WhenAny(Task.Delay(seconds * 1000), first ? (Task)found.Task : Task.Delay(-1));
        w.Stop();
        if (w.Status == BluetoothLEAdvertisementWatcherStatus.Aborted)
        {
            Console.WriteLine("ERROR scanner aborted (is this PC's BLE radio on?)");
            return 2;
        }
        return (prefix == null || found.Task.IsCompleted) ? 0 : 1;
    }

    /* GATT client. */

    static async Task<BluetoothLEDevice> Open(string address)
    {
        BluetoothLEDevice d = await W(BluetoothLEDevice.FromBluetoothAddressAsync(ParseAddress(address)));
        if (d == null) throw new InvalidOperationException("no device at " + address);
        return d;
    }

    static async Task<Dictionary<Guid, GattCharacteristic>> Characteristics(BluetoothLEDevice d, bool print)
    {
        Dictionary<Guid, GattCharacteristic> map = new Dictionary<Guid, GattCharacteristic>();
        GattDeviceServicesResult sr = null;
        for (int attempt = 0; attempt < 3; attempt++)
        {
            sr = await W(d.GetGattServicesAsync(BluetoothCacheMode.Uncached));
            if (sr.Status == GattCommunicationStatus.Success) break;
            await Task.Delay(1000);
        }
        if (sr.Status != GattCommunicationStatus.Success)
            throw new InvalidOperationException("service discovery: " + sr.Status);
        foreach (GattDeviceService svc in sr.Services)
        {
            if (print) Console.WriteLine("SVC " + svc.Uuid);
            GattCharacteristicsResult cr = await W(svc.GetCharacteristicsAsync(BluetoothCacheMode.Uncached));
            if (cr.Status != GattCommunicationStatus.Success) continue;
            foreach (GattCharacteristic c in cr.Characteristics)
            {
                if (print) Console.WriteLine("CHR " + c.Uuid + " props=" + c.CharacteristicProperties);
                if (!map.ContainsKey(c.Uuid)) map[c.Uuid] = c;
            }
        }
        return map;
    }

    static async Task<int> Gatt(List<string> a)
    {
        if (a.Count == 0) { Console.WriteLine("ERROR gatt needs an address"); return 2; }
        int seconds = int.Parse(Opt(a, "--seconds", "5"));
        int failures = 0;
        using (BluetoothLEDevice d = await Open(a[0]))
        {
            Dictionary<Guid, GattCharacteristic> chars = await Characteristics(d, Flag(a, "--list"));
            Console.WriteLine("CONNECTED name=\"" + d.Name + "\" status=" + d.ConnectionStatus +
                              " characteristics=" + chars.Count);

            foreach (string u in Opts(a, "--read"))
            {
                GattCharacteristic c;
                if (!chars.TryGetValue(ParseUuid(u), out c)) { Console.WriteLine("READ " + u + " status=Missing"); failures++; continue; }
                GattReadResult r = await W(c.ReadValueAsync(BluetoothCacheMode.Uncached));
                if (r.Status == GattCommunicationStatus.Success)
                {
                    byte[] v = Bytes(r.Value);
                    Console.WriteLine("READ " + u + " status=Success hex=" + Hex(v) + " text=\"" + Printable(v) + "\"");
                }
                else
                {
                    Console.WriteLine("READ " + u + " status=" + r.Status +
                                      (r.ProtocolError.HasValue ? " att=0x" + r.ProtocolError.Value.ToString("x2") : ""));
                    failures++;
                }
            }

            foreach (string w in Opts(a, "--write"))
            {
                string[] kv = w.Split('=');
                GattCharacteristic c;
                if (!chars.TryGetValue(ParseUuid(kv[0]), out c)) { Console.WriteLine("WRITE " + kv[0] + " status=Missing"); failures++; continue; }
                GattWriteResult r = await W(c.WriteValueWithResultAsync(Buf(FromHex(kv[1])), GattWriteOption.WriteWithResponse));
                Console.WriteLine("WRITE " + kv[0] + " status=" + r.Status +
                                  (r.ProtocolError.HasValue ? " att=0x" + r.ProtocolError.Value.ToString("x2") : ""));
                if (r.Status != GattCommunicationStatus.Success) failures++;
            }

            string n = Opt(a, "--notify", null);
            if (n != null)
            {
                GattCharacteristic c;
                if (!chars.TryGetValue(ParseUuid(n), out c)) { Console.WriteLine("NOTIFY " + n + " status=Missing"); return 1; }
                int count = 0;
                c.ValueChanged += (s, e) =>
                {
                    count++;
                    Console.WriteLine("NOTIFY " + n + " hex=" + Hex(Bytes(e.CharacteristicValue)));
                };
                GattCommunicationStatus st = await W(c.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue.Notify));
                Console.WriteLine("SUBSCRIBE " + n + " status=" + st);
                if (st != GattCommunicationStatus.Success) return 1;
                await Task.Delay(seconds * 1000);
                await W(c.WriteClientCharacteristicConfigurationDescriptorAsync(
                    GattClientCharacteristicConfigurationDescriptorValue.None));
                Console.WriteLine("NOTIFIED count=" + count);
                if (count == 0) failures++;
            }
        }
        return failures == 0 ? 0 : 1;
    }

    /* Pairing. */

    static async Task<int> Pair(List<string> a)
    {
        if (a.Count == 0) { Console.WriteLine("ERROR pair needs an address"); return 2; }
        string pin = Opt(a, "--pin", null);
        using (BluetoothLEDevice d = await Open(a[0]))
        {
            DeviceInformationPairing p = d.DeviceInformation.Pairing;
            if (p.IsPaired) { Console.WriteLine("PAIR status=AlreadyPaired"); return 0; }
            p.Custom.PairingRequested += (s, e) =>
            {
                Console.WriteLine("PAIRING kind=" + e.PairingKind + (string.IsNullOrEmpty(e.Pin) ? "" : " pin=" + e.Pin));
                if (e.PairingKind == DevicePairingKinds.ProvidePin && pin != null) e.Accept(pin);
                else e.Accept();
            };
            DevicePairingResult r = await W(p.Custom.PairAsync(
                DevicePairingKinds.ConfirmOnly | DevicePairingKinds.ConfirmPinMatch |
                DevicePairingKinds.DisplayPin | DevicePairingKinds.ProvidePin,
                DevicePairingProtectionLevel.Encryption));
            Console.WriteLine("PAIR status=" + r.Status + " level=" + r.ProtectionLevelUsed);
            return (r.Status == DevicePairingResultStatus.Paired || r.Status == DevicePairingResultStatus.AlreadyPaired) ? 0 : 1;
        }
    }

    static async Task<int> Unpair(List<string> a)
    {
        if (a.Count == 0) { Console.WriteLine("ERROR unpair needs an address"); return 2; }
        using (BluetoothLEDevice d = await Open(a[0]))
        {
            if (!d.DeviceInformation.Pairing.IsPaired) { Console.WriteLine("UNPAIR status=NotPaired"); return 0; }
            DeviceUnpairingResult r = await W(d.DeviceInformation.Pairing.UnpairAsync());
            Console.WriteLine("UNPAIR status=" + r.Status);
            return r.Status == DeviceUnpairingResultStatus.Unpaired ? 0 : 1;
        }
    }

    /* This PC as a peripheral. */

    static async Task<int> Advertise(List<string> a)
    {
        int seconds = int.Parse(Opt(a, "--seconds", "20"));
        BluetoothLEAdvertisementPublisher pub = new BluetoothLEAdvertisementPublisher();
        BluetoothLEManufacturerData m = new BluetoothLEManufacturerData();
        m.CompanyId = 0xFFFF;   /* reserved for testing */
        m.Data = Buf(FromHex(Opt(a, "--mfg", "4e36")));
        pub.Advertisement.ManufacturerData.Add(m);
        pub.StatusChanged += (s, e) => Console.WriteLine("ADVERTISE status=" + e.Status + " error=" + e.Error);
        pub.Start();
        await Task.Delay(seconds * 1000);
        pub.Stop();
        return 0;
    }

    static async Task<int> Server(List<string> a)
    {
        int seconds = int.Parse(Opt(a, "--seconds", "30"));
        GattServiceProviderResult spr = await W(GattServiceProvider.CreateAsync(BenchService));
        if (spr.Error != BluetoothError.Success) { Console.WriteLine("ERROR server: " + spr.Error); return 2; }
        GattServiceProvider prov = spr.ServiceProvider;

        GattLocalCharacteristicParameters p = new GattLocalCharacteristicParameters();
        p.CharacteristicProperties = GattCharacteristicProperties.Read | GattCharacteristicProperties.Write |
                                     GattCharacteristicProperties.Notify;
        p.ReadProtectionLevel = GattProtectionLevel.Plain;
        p.WriteProtectionLevel = GattProtectionLevel.Plain;
        p.UserDescription = "n6 bench value";
        GattLocalCharacteristicResult cr = await W(prov.Service.CreateCharacteristicAsync(BenchValue, p));
        if (cr.Error != BluetoothError.Success) { Console.WriteLine("ERROR characteristic: " + cr.Error); return 2; }
        GattLocalCharacteristic ch = cr.Characteristic;
        byte[] value = Encoding.ASCII.GetBytes("n6-pc-ok");

        ch.ReadRequested += async (s, e) =>
        {
            Windows.Foundation.Deferral def = e.GetDeferral();
            GattReadRequest req = await W(e.GetRequestAsync());
            req.RespondWithValue(Buf(value));
            Console.WriteLine("SERVED read by " + e.Session.DeviceId.Id);
            def.Complete();
        };
        ch.WriteRequested += async (s, e) =>
        {
            Windows.Foundation.Deferral def = e.GetDeferral();
            GattWriteRequest req = await W(e.GetRequestAsync());
            byte[] v = Bytes(req.Value);
            Console.WriteLine("SERVED write hex=" + Hex(v) + " text=\"" + Printable(v) + "\"");
            if (req.Option == GattWriteOption.WriteWithResponse) req.Respond();
            def.Complete();
        };

        GattServiceProviderAdvertisingParameters ap = new GattServiceProviderAdvertisingParameters();
        ap.IsConnectable = true;
        ap.IsDiscoverable = true;
        prov.AdvertisementStatusChanged += (s, e) => Console.WriteLine("SERVER status=" + e.Status + " error=" + e.Error);
        prov.StartAdvertising(ap);
        await Task.Delay(seconds * 1000);
        prov.StopAdvertising();
        return 0;
    }

    /* Wi-Fi Direct group owner, legacy mode. */

    static async Task<int> WfdAp(List<string> a)
    {
        string ssid = Opt(a, "--ssid", null), key = Opt(a, "--key", null);
        int seconds = int.Parse(Opt(a, "--seconds", "300"));
        if (ssid == null || key == null) { Console.WriteLine("ERROR wfd-ap needs --ssid and --key"); return 2; }

        WiFiDirectAdvertisementPublisher pub = new WiFiDirectAdvertisementPublisher();
        pub.Advertisement.IsAutonomousGroupOwnerEnabled = true;
        pub.Advertisement.LegacySettings.IsEnabled = true;
        pub.Advertisement.LegacySettings.Ssid = ssid;
        PasswordCredential cred = new PasswordCredential();
        cred.Password = key;
        pub.Advertisement.LegacySettings.Passphrase = cred;
        pub.StatusChanged += (s, e) => Console.WriteLine("WFD status=" + e.Status + " error=" + e.Error);

        /* Legacy stations join without a Wi-Fi Direct connection request, but
         * the listener must exist for the group to accept anyone. */
        WiFiDirectConnectionListener listener = new WiFiDirectConnectionListener();
        listener.ConnectionRequested += (s, e) => Console.WriteLine("WFD request from " + e.GetConnectionRequest().DeviceInformation.Name);

        pub.Start();
        for (int i = 0; i < 20 && pub.Status != WiFiDirectAdvertisementPublisherStatus.Started; i++)
            await Task.Delay(250);
        if (pub.Status != WiFiDirectAdvertisementPublisherStatus.Started)
        {
            Console.WriteLine("ERROR group owner did not start: " + pub.Status);
            return 2;
        }

        /* The group owner's own address, once Windows has assigned it. */
        for (int i = 0; i < 40; i++)
        {
            string ip = GroupOwnerAddress();
            if (ip != null) { Console.WriteLine("WFD ip=" + ip); break; }
            await Task.Delay(250);
        }
        Console.WriteLine("WFD ready ssid=" + ssid);
        await Task.Delay(seconds * 1000);
        pub.Stop();
        Console.WriteLine("WFD stopped");
        return 0;
    }

    static string GroupOwnerAddress()
    {
        foreach (NetworkInterface ni in NetworkInterface.GetAllNetworkInterfaces())
        {
            if (ni.OperationalStatus != OperationalStatus.Up) continue;
            if (ni.Description.IndexOf("Wi-Fi Direct", StringComparison.OrdinalIgnoreCase) < 0) continue;
            foreach (UnicastIPAddressInformation u in ni.GetIPProperties().UnicastAddresses)
                if (u.Address.AddressFamily == AddressFamily.InterNetwork) return u.Address.ToString();
        }
        return null;
    }
}
