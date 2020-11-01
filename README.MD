
# Debug messages monitor for Qualcomm cellular modems

## General information

This program allows to monitor debug messages of Qualcomm cellular modems firmware in real time. It talks to the baseband firmware over the [DIAG protocol](https://osmocom.org/projects/quectel-modems/wiki/Diag) serial port, you can use it with rooted Android phone or USB dongle.

Its output might be useful for various purposes like modem firmware reverse engineering, detecting of [IMSI catchers](https://en.wikipedia.org/wiki/IMSI-catcher), monitoring of commercial cellular networks, troubleshooting of DIY cellular networks and others.

To access DIAG interface serial port this program uses [pySerial](https://pypi.org/project/pyserial/):

```
$ sudo pip install pyserial
```

Usage: 

```
$ sudo qc_debug_monitor.py -d <device_path> [options]
```

Available options:

* -d, --device-name &lt;device_path&gt; – DIAG serial port device path.

* -b, --baudrate &lt;baudrate&gt; – Serial port baudrate, 115200 is used by default.

* -o, --output-file &lt;file_path&gt; – Output file path to record captured messages.

* -u, --output-dir &lt;dir_path&gt; – Output directory path to record captured messages into the separate log files for each modem subsystem (see below for more information).
                        
* -f, --find-subsystems – Find all valid subsystem IDs of the target modem.
                        
* -s, --subsystems &lt;id_list&gt; – Subsystem IDs list to capture debug messages for, by default `qc_debug_monitor.py` captures messages of all known subsystems.
                        
* -t, --show-terse – Show not decoded terse messages

* -a, --hash-db &lt;db_file_path&gt; – Hash DB file path to decode terse messages.
  
* --daemon – Run in the background.

* --stop – Stop background process.

Modem firmware has two types of debug messages: regular text message and terse message that consists from 32-bit hash of the format string plus its arguments. Hashes are used for the most frequent messages to save serial port bandwidth. In order to decode terse messages into the human readable text you need to specify hash DB file path with `--hash-db` option.

Hash DB file with hashes of over the 300000 debug messages of Qualcomm firmware collected from various sources [can be downloaded here](https://mega.nz/file/bR8wWK6I#8Yo9naeAe8d-jdW30D8u7q_N2Xcl54TebyQMOZNtzjk).

Here you can see some demo:

[![asciicast](https://asciinema.org/a/JKsQuW8BurczhFpRYT8Z7tSD4.svg)](https://asciinema.org/a/JKsQuW8BurczhFpRYT8Z7tSD4)

## Modem subsystems

Modem firmware consists from dozens of subsystems responsible for various tasks: power management, AT commands handling, Qualcomm MSM interface, cryptography, different layers of cellular protocols and much, much more.

When `--output-file` is specified – all log messages will be written into the single log file. When `--output-dir` is specified – `qc_debug_monitor.py` will create separate log files for each subsystem identified by its 16-bit SSID (subsystem ID).

To find all supported subsystems of the target modem you can use `--find-subsystems` option which brute-forces and prints available SSID ranges. Here's an example of using this option with [Quectel EC25](https://www.quectel.com/product/ec25.htm) cellular modem module based on Qualcomm MDM9607 chip:

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

In `examples` folder [you can find](https://github.com/Cr4sh/qc_debug_monitor/tree/master/examples) debug messages logs of modem module mentioned above captured during its connection to the [srsLTE](https://github.com/srsLTE/srsLTE) based [eNodeB](https://en.wikipedia.org/wiki/Home_eNodeB) running on downlink EARFCN 3400 (band 7, 2685 MHz). Log file `examples/full.log` contains all subsystems messages, while `examples/subsystem` folder contains individual log files for each subsystem.

Qualcomm modems firmware is extremely talkative, usually you can get 3-4 MB of debug messages log within less than minute of normal operation.

Some interesting subsystems SSIDs as it seen on Quectel EC25, subsystem description is based on my guess and obviously might be inaccurate:

* [0x0005](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0005.log) – Received Signal Strength Indicator (RSSI) measurements.

* [0x000e](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_000e.log) – RF transceiver control subsystem.

* [0x0015](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0015.log) – Multi Mode Generic SIM Driver Interface (MMSGDI) subsystem.

* [0x0020](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0020.log) – Temperature Compensated Crystal Oscillator (TCXO) control subsystem.

* [0x0040](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0040.log) – Power management related subsystem.

* [0x0041](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_0041.log) – Qualcomm MSM Interface (QMI) related subsystem.

* [0x1394](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_1394.log) – AT commands handling subsystem.

* [0x13a1](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_13a1.log) – PDN connectivity subsystem;

* [0x251c](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_251c.log) – LTE security and encryption subsystem.

* [0x251d](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_251d.log) – LTE Radio Resource Control (RRC) layer.

* [0x2521](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2521.log) – LTE Radio Link Control (RLC) layer, uplink.

* [0x2522](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2522.log) – LTE Radio Link Control (RLC) layer, downlink.

* [0x2523](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2523.log) – LTE Packet Data Convergence Protocol (PDCP) layer, uplink.

* [0x2524](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2524.log) – LTE Packet Data Convergence Protocol (PDCP) layer, downlink.

* [0x2525](https://github.com/Cr4sh/qc_debug_monitor/blob/master/examples/subsystems/ssid_2525.log) – LTE serving cell measurement.


## Examples

Monitor all messages and log them into the single file:

```
$ sudo qc_debug_monitor.py -d /dev/ttyUSB0 -a hash_db.txt -o output.log 
```

Monitor all messages and log them into the separate files for each subsystem:

```
$ sudo qc_debug_monitor.py -d /dev/ttyUSB0 -a hash_db.txt -u ./output
```

Monitor messages of the single subsystem with SSID 0x251d:

```
$ sudo qc_debug_monitor.py -d /dev/ttyUSB0 -a hash_db.txt -s 0x251d
```

Monitor messages of subsystems with SSID from 0x1388 to 0x1450 and from 0x251c to 0x25e4:

```
$ sudo qc_debug_monitor.py -d /dev/ttyUSB0 -a hash_db.txt -s "0x1388:0x1450 0x251c:0x25e4"
```

---

Developed by:<br/>
Dmytro Oleksiuk (aka Cr4sh)<br/>

cr4sh0@gmail.com<br/>
http://blog.cr4.sh
