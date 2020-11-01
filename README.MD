
# Debug messages monitor for Qualcomm cellular modems

## General information

This program allows to monitor debug messages of Qualcomm cellular modems firmware in real time. It talks to the baseband firmware over the <a href="https://osmocom.org/projects/quectel-modems/wiki/Diag">DIAG protocol</a> serial port, you can use it with rooted Android phone or USB dongle.

Its output might be useful for various purposes like modem firmware reverse engineering, detecting <a href="https://en.wikipedia.org/wiki/IMSI-catcher">IMSI catchers</a>, monitoring commercial cellular networks, troubleshooting DIY cellular networks and others.

To access DIAG interface serial port this program uses <a href="https://pypi.org/project/pyserial/">pySerial</a>:

```
$ sudo pip install pyserial
```

Usage: 

```
$ sudo qc_debug_monitor.py -d <device_name> [options]
```

Available options:

* -d, --device-name <device_name> – DIAG serial port device name;

* -b, --baudrate <baudrate> – Serial port baudrate, 115200 is used by default;

* -o, --output-file <file_path> – Output file path to record captured messages;

* --output-dir <dir_path> – Output directory path to record captured messages into the separate log files for each modem subsystem (see below for more information);
                        
* -f, --find-subsystems – Find all valid subsystem IDs of the target modem;
                        
* -s, --subsystems <id_list> – Subsystem IDs list to capture debug messages for, by default `qc_debug_monitor.py` captures messages of all known subsystems;
                        
* -t, --show-terse – Show not decoded terse messages;

* --hash-db <db_file_path> – Hash DB file path to decode terse messages;
  
* --daemon – Run in the background;

* --stop – Stop background process;

Modem firmware has two types of debug messages: regular text message and terse message that consists from 32-bit hash of the format string plus its arguments. Hashes are used for the most frequent messages to save serial port bandwidth. In order to decode terse messages into the human readable text you need to specify a hash DB file with `--hash-db` option.

Hash DB file with hashes of over the 300000 debug messages of Qualcomm firmware collected from various sources <a href="https://mega.nz/file/bR8wWK6I#8Yo9naeAe8d-jdW30D8u7q_N2Xcl54TebyQMOZNtzjk">can be downloaded here</a>.

Here you can see some demo:

[![asciicast](https://asciinema.org/a/JKsQuW8BurczhFpRYT8Z7tSD4.svg)](https://asciinema.org/a/JKsQuW8BurczhFpRYT8Z7tSD4)

## Modem subsystems

Modem firmware consists from dozens of subsystems responsible for various tasks: power management, AT commands handling, Qualcomm MSM interface, cryptography, different layers of cellular protocols and much, much more.

When `--output-file` is specified – all log messages will be written into the single log file. When `--output-dir` is specified – `qc_debug_monitor.py` will create separate log files for each subsystem identified by its 16-bit SSID (subsystem ID).

To find all supported subsystems of the target modem you can use `--find-subsystems` option which brute-forces and prints available SSID ranges. Here's an example of using this option with <a href="https://www.quectel.com/product/ec25.htm">Quectel EC25</a> cellular modem module based on Qualcomm MDM9607 chip:

```
$ sudo ./qc_debug_monitor.py -d /dev/ttyUSB0 --find-subsystems
[+] Opening /dev/ttyUSB0
[+] Device model: 9607.gen
[+] Revision: 100
[+] Finding all available subsystem IDs, it may take a while...

0x0000:0x00c8
0x01f4:0x02bc
0x03e8:0x04b0
0x07d0:0x0898
0x0bb8:0x0c80
0x0fa0:0x1068
0x1194:0x12c0
0x1388:0x1450
0x157c:0x1644
0x1770:0x1838
0x1964:0x1a2c
0x1b58:0x1ce8
0x1f40:0x2008
0x2134:0x21fc
0x2328:0x23f0
0x251c:0x25e4

[+] DONE
```

By default `qc_debug_monitor.py` monitors all known subsystems messages specified in `KNOWN_SUBSYSTEMS` global variable – you can put there SSID ranges found by running the program with `--find-subsystems` option.

In `examples` folder <a href="https://github.com/Cr4sh/qc_debug_monitor/tree/master/examples">you can find</a> debug messages logs of modem module mentioned above captured during its connection to the <a href="https://github.com/srsLTE/srsLTE">srsLTE</a> based <a href="https://en.wikipedia.org/wiki/Home_eNodeB">eNodeB</a> running on downlink EARFCN 3400 (band 7, 2685 MHz). Log file `examples/full.log` contains all subsystems messages, while `examples/subsystem` folder contains individual log files for each subsystem.

Qualcomm modems firmware is extremely talkative, usually you can get 3-4 MB of debug messages log within less than minute of normal operation.

Some interesting subsystems SSIDs as it seen on Quectel EC25, subsystem description is based on my guess and obviously might be inaccurate:

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0005.log">0x0005</a> – Received Signal Strength Indicator (RSSI) measurements;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_000e.log">0x000e</a> – RF transceiver control subsystem;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0015.log">0x0015</a> – Multi Mode Generic SIM Driver Interface (MMSGDI) subsystem;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0020.log">0x0020</a> – Temperature Compensated Crystal Oscillator (TCXO) control subsystem;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0040.log">0x0040</a> – Power management related subsystem;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0041.log">0x0041</a> – Qualcomm MSM Interface (QMI) related subsystem;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_1394.log">0x1394</a> – AT commands handling subsystem;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_13a1.log">0x13a1</a> – PDN connectivity subsystem;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_251c.log">0x251c</a> – LTE security and encryption subsystem;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_251d.log">0x251d</a> – LTE Radio Resource Control (RRC) layer;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2521.log">0x2521</a> – LTE Radio Link Control (RLC) layer, uplink;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2522.log">0x2522</a> – LTE Radio Link Control (RLC) layer, downlink;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2523.log">0x2523</a> – LTE Packet Data Convergence Protocol (PDCP) layer, uplink;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2524.log">0x2524</a> – LTE Packet Data Convergence Protocol (PDCP) layer, downlink;

* <a href="https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2525.log">0x2525</a> – LTE serving cell measurement;


## Examples

Monitor all messages and log them into the single file:

```
$ sudo qc_debug_monitor.py -d /dev/ttyUSB0 --hash-db hash_db.txt -o output.log 
```

Monitor all messages and log them into the separate files for each subsystem:

```
$ sudo qc_debug_monitor.py -d /dev/ttyUSB0 --hash-db hash_db.txt --output-dir ./output
```

Monitor messages of the single subsystem with SSID 0x251d:

```
$ sudo qc_debug_monitor.py -d /dev/ttyUSB0 --hash-db hash_db.txt --subsystems 0x251d
```

Monitor messages of subsystems with SSID from 0x1388 to 0x1450 and from 0x251c to 0x25e4:

```
$ sudo qc_debug_monitor.py -d /dev/ttyUSB0 --hash-db hash_db.txt --subsystems "0x1388:0x1450 0x251c:0x25e4"
```

---

Developed by:<br/>
Dmytro Oleksiuk (aka Cr4sh)<br/>

cr4sh0@gmail.com<br/>
http://blog.cr4.sh
