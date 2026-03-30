# hokuyo_control_server

ROS HTTP bridge se sjednocenym API pro:
- realneho robota na ROS Indigo
- Gazebo simulator na ROS Noetic

## API endpointy

- `GET /health`
- `POST /cmd_vel`
  - payload: `{ "linear_x": 0.2, "angular_z": 0.4 }`
- `GET /sensors/lidar`
- `GET /sensors/camera`
- `GET /objects`
- `GET /battery`
- `GET /markers` (QR + ArUco detekce z kamery)
- `GET /velocity` (posledni aplikovany `cmd_vel`)
- `POST /led` (Kobuki LED1/LED2)
- `GET /led`
- `POST /sound` (jen `kobuki_msgs/Sound`)
- `GET /sound`
- `POST /tts` (syntéza řeči přes `rosrun sound_play say.py`)
- `GET /tts`

## Object data format

Endpoint `/objects` vraci posledni zpravu z topicu `/results` typu `std_msgs/String`.
JSON je ulozene v `msg.data`, ale API z nej vraci jen pole `detections`:

```json
{
  "data": [
    {
      "center_x": 0.4731,
      "center_y": 0.7161,
      "width": 0.2717,
      "height": 0.4145,
      "score": 0.8889358639717102,
      "class": 0,
      "class_name": "person"
    }
  ]
}
```

Server vraci posledni data jen po omezenou dobu (`objects_max_age`, default `1.0s`).
Pokud neprijde nova zprava na `objects_topic` dele nez timeout, endpoint vrati prazdne pole.
Souřadnice i rozměry jsou normalizovane do rozsahu `0..1` pro obraz 640x480.

## Spusteni - Indigo (real robot)

### Pres rosrun

```bash
rosrun hokuyo_control_server robot_http_server_indigo.py _port:=8080
```

### Pres roslaunch

```bash
roslaunch hokuyo_control_server robot_http_server_indigo.launch port:=8080
```

Vychozi topics:
- `cmd_vel_topic=/mobile_base/commands/velocity`
- `lidar_topic=/scan`
- `camera_topic=/usb_cam/image_raw/compressed`
- `camera_encoding=compressed`
- `objects_topic=/results`
- `objects_max_age=1.0` (maximalni stari posledni prijate zpravy pro `/objects`)
- `battery_topic=/battery_state`
- `battery_mode=laptop_sysfs` (cteni z `/sys/class/power_supply/BAT*` na notebooku)
- `max_speed=0.2` (spolecny fallback limit)
- `max_linear_speed` (default `max_speed`)
- `max_angular_speed=0.5`
- `hold_cmd_vel=true` (periodicky publish na `cmd_vel`)
- `cmd_vel_hold_rate=10.0` (Hz)
- `cmd_vel_timeout=1.0` (sekundy bez noveho `POST /cmd_vel` -> server posila nulovou rychlost)
- `led1_topic=/mobile_base/commands/led1`
- `led2_topic=/mobile_base/commands/led2`
- `sound_topic=/robotsound`
- `tts_mode=say_script` (spousti `rosrun sound_play say.py "<text>"`)
- `kobuki_sound_topic=/mobile_base/commands/sound`
- `detect_markers=true`
- `marker_min_interval=0.2` (sekundy)
- `aruco_dict_name=DICT_4X4_50`

## Spusteni - Noetic Gazebo

### Pres rosrun

```bash
rosrun hokuyo_control_server robot_http_server_noetic_gazebo.py _port:=8081
```

### Pres roslaunch

```bash
roslaunch hokuyo_control_server robot_http_server_noetic_gazebo.launch port:=8081
```

Vychozi topics:
- `cmd_vel_topic=/cmd_vel`
- `lidar_topic=/scan`
- `camera_topic=/camera/image_raw`
- `camera_encoding=raw`
- `battery_topic=/battery`
- `max_speed=0.2` (spolecny fallback limit)
- `max_linear_speed` (default `max_speed`)
- `max_angular_speed=0.5`
- `battery_mode=float32` (`std_msgs/Float32`, hodnota 0..1)
- `hold_cmd_vel=true` (periodicky publish na `cmd_vel`)
- `cmd_vel_hold_rate=10.0` (Hz)
- `cmd_vel_timeout=1.0` (sekundy bez noveho `POST /cmd_vel` -> server posila nulovou rychlost)
- `detect_markers=false` (lze zapnout i pro Gazebo)

Noetic/Gazebo varianta vraci na `/objects` dummy data pro testy.

## Jednotny klient

`src/http_api_client.py` obsahuje klienta `RobotHttpClient`:

```python
from http_api_client import RobotHttpClient

client = RobotHttpClient("http://127.0.0.1:8080")
client.send_cmd_vel(0.2, 0.1)
print(client.get_lidar())
print("obstacle_report =", client.get_obstacle_report())
print(client.get_camera())
print(client.get_objects())
print(client.get_battery())
print(client.get_markers())
print(client.get_velocity())
client.send_cmd_vel(0.0, 0.0)
client.set_led(1, "GREEN")
client.play_kobuki_sound("BUTTON")
client.say("Ahoj svete")
```

Jednoduchy HTML navod pro studenty:
- `docs/client_api_guide.html`

Priklad CLI:

```bash
rosrun hokuyo_control_server example_client_usage.py --url http://127.0.0.1:8080 --linear 0.1 --angular 0.3
```

Zobrazeni kamery v OpenCV okne:

```bash
rosrun hokuyo_control_server example_camera_viewer.py --url http://127.0.0.1:8080 --fps 10
```

## Launch soubory

- `launch/robot_http_server_indigo.launch`
- `launch/robot_http_server_noetic_gazebo.launch`

Oba soubory maji argumenty pro host/port a topicy. Indigo navic pridava `led1_topic`, `led2_topic`, `kobuki_sound_topic`, napr.:

```bash
roslaunch hokuyo_control_server robot_http_server_indigo.launch \
  cmd_vel_topic:=/mobile_base/commands/velocity \
  lidar_topic:=/hokuyo/scan \
  battery_topic:=/power/battery_state
```

## Marker endpoint (`/markers`)

Polling endpoint vraci posledni detekci markeru:

```json
{
  "data": {
    "stamp": 1700000000.12,
    "enabled": true,
    "qr_codes": [
      { "data": "ROOM-A", "polygon": [[10, 20], [110, 18], [112, 120], [9, 121]] }
    ],
    "aruco_markers": [
      { "id": 23, "corners": [[200, 120], [260, 120], [260, 180], [200, 180]] }
    ]
  }
}
```

Klient pak muze periodicky volat `get_markers()` a rozhodovat, jestli robot vidi cilovy kod.
Na starem Ubuntu 14.04 / Pythonu 2.7 bez `cv2.aruco` bezi jednodussi fallback, ktery vraci kandidaty s `id = -1` a jejich rohy.

## Lidar helpery v klientovi

V `RobotHttpClient` jsou metody:
- `analyze_lidar(scan_data)`
- `get_obstacle_report()` (sama si nacte lidar)

Vystup obsahuje:
- `front/front_left/front_right/left/right/back` (vzdalenosti v metrech)

## Velocity endpoint (`/velocity`)

Vraci posledni aplikovany command na `cmd_vel`. Pokud vyprsi `cmd_vel_timeout`,
server prepne posledni rychlost na nulu.

```json
{
  "data": {
    "source": "cmd_vel",
    "stamp": 1700000000.12,
    "linear": {"x": 0.05, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": -0.2}
  }
}
```

## LED endpoint (`/led`)

Nastaveni LED:

```json
POST /led
{ "led_index": 1, "value": "GREEN" }
```

`value` muze byt `BLACK|GREEN|ORANGE|RED` nebo `0..3`.

## Sound endpoint (`/sound`)

Kobuki zvuky (`kobuki_msgs/Sound`):

```json
POST /sound
{ "value": "BUTTON" }
```

`value`: `ON|OFF|RECHARGE|BUTTON|ERROR|CLEANINGSTART|CLEANINGEND` nebo `0..6`.

## TTS endpoint (`/tts`)

Syntéza řeči přes `sound_play` (na Indigu defaultně přes `rosrun sound_play say.py`):

```json
POST /tts
{ "text": "Ahoj svete", "command": 1 }
```

`command`:
- `1` = `PLAY_ONCE`
- `2` = `PLAY_START`
- `0` = `PLAY_STOP`

Pozn.: V `tts_mode=say_script` se předává text do `say.py`; `command` je přijat pro kompatibilitu API.

## Cmd_vel watchdog

Server publikuje na `cmd_vel` periodicky (`hold_cmd_vel`, `cmd_vel_hold_rate`), ale jen po dobu,
kdy klient pravidelne posila nove `POST /cmd_vel` (v klientovi `RobotHttpClient` se to dela interne po `send_cmd_vel`).
Po vyprseni `cmd_vel_timeout` server automaticky publikuje nulovou rychlost (`0, 0`).

## Poznamky

- Kamera se vraci jako base64 (`data_base64`), aby endpoint fungoval bez ohledu na typ zpravy.
- Pokud realny robot neposila `face_detector_msgs/DetectedFaces`, endpoint vraci prazdne pole.
- Topic names lze upravit pres ROS parametry `~...`.
