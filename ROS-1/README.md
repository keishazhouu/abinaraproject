# KPP Introduction to ROS: ROS 1 + MuJoCo

Integrasi ROS 1 (Noetic) dengan simulasi robot arm 4-DOF + gripper dari KPP Modul 2 (MuJoCo, C++).
Ujung gripper (end-effector) digerakkan membentuk lintasan **segitiga** lewat pola publisher/subscriber.
**Bonus:** dua robot identik dalam satu simulasi, bergerak sinkron lewat komunikasi ROS dan dapat
diidentifikasi serta dikontrol terpisah.

## Arsitektur

```
 triangle_trajectory_node                 mujoco_sim_node
 (lintasan segitiga + IK)                 (Modul 2: sim + viewer + control)
        |  /arm1/joint_cmd  ------------->  RosBridge -> d->ctrl -> aktuator MuJoCo
        |                                   RosBridge <- state
        |                                      |-- /arm1/joint_states
        |  /arm1/ee_target (debug)             '-- /arm1/ee_position
        ^
        |  /sync/phase  (hanya mode dual-arm)
   sync_node (master clock)
```

| Node | Fungsi |
|---|---|
| `mujoco_sim_node` | Menjalankan simulasi + viewer Modul 2. Menerima target joint, mem-publish state dan posisi end-effector tiap robot. |
| `triangle_trajectory_node` | Membuat titik di keliling segitiga, menghitung inverse kinematics, mem-publish target joint. |
| `sync_node` | Mem-publish fase global 0..1 agar banyak robot bergerak serempak (bonus). |

## Topic

`<robot>` = `arm1` (dan `arm2` pada bonus).

| Topic | Tipe message | Arah | Isi |
|---|---|---|---|
| `/<robot>/joint_cmd` | `std_msgs/Float64MultiArray` | trajectory → sim | `[hip, shoulder, elbow, wrist]` rad. Opsional 2 nilai tambahan `[left, right]` untuk gripper |
| `/<robot>/joint_states` | `sensor_msgs/JointState` | sim → publik | posisi, kecepatan, gaya 6 joint |
| `/<robot>/ee_position` | `geometry_msgs/PointStamped` | sim → publik | posisi ujung gripper, frame base robot (meter) |
| `/<robot>/ee_target` | `geometry_msgs/PointStamped` | trajectory → publik | target ee pada lintasan (frame base), untuk debug/perbandingan |
| `/<robot>/enable` | `std_msgs/Bool` | publik → trajectory | `false` = pause trajectory |
| `/sync/phase` | `std_msgs/Float64` | sync → trajectory | fase 0..1 (mode dual-arm) |

## Alasan desain node dan topic

- **Simulator dan perencana gerak dipisah.** Simulator hanya tahu "terima target joint, jalankan fisika, laporkan state".
  Perencana hanya tahu geometri dan waktu. Keduanya bisa diganti atau diuji sendiri (mis. lintasan lingkaran tanpa
  menyentuh simulator, atau `rostopic pub` menggantikan perencana).
- **Topic, bukan service.** Data berupa aliran kontinu (100 Hz) dan pengirim tidak perlu menunggu balasan;
  yang berlaku selalu perintah terbaru, yang lama boleh terlewat.
- **Modul 2 dipertahankan.** `sim/`, `callback/`, dan `control/` dipakai kembali; ROS ditambahkan sebagai satu modul
  baru (`ros_bridge/`) sehingga kode fisika/viewer tidak tercampur dengan kode komunikasi.
- **Satu loop, satu thread.** `ros::spinOnce()` dipanggil di loop utama, jadi callback ROS, fisika, dan render
  berjalan berurutan di satu thread. Tidak ada race condition atau mutex yang perlu dijelaskan.
- **Namespace per robot** (`/arm1/...`, `/arm2/...`): node yang sama dijalankan dua kali pada namespace berbeda,
  sehingga tiap robot punya topic sendiri dan bisa dikontrol terpisah tanpa mengubah kode.
- **Master clock `/sync/phase`.** Sinkronisasi dilakukan lewat komunikasi ROS. Semua trajectory node membaca fase yang
  sama sehingga serempak tanpa saling mengenal. `phase_offset` per robot memungkinkan pola lain (0.5 = berlawanan fase).
- **Satu `mujoco_sim_node` untuk semua robot**, karena kedua robot harus berada di lingkungan simulasi yang sama
  (satu `mjModel`/`mjData`).

## Kinematika dan lintasan

Konstanta diturunkan dari `models/arm_final.xml`. Lengan bergerak pada bidang vertikal `(r, z)` yang diputar oleh `hip`.
Sudut kumulatif link `θ` dari sumbu z (link menunjuk ke `(sin θ, cos θ)`):

```
θ1 = 1.0  + q_shoulder            (quat upper_arm = 1.0 rad terhadap y)
θ2 = 2.15 + q_shoulder + q_elbow  (quat lower_arm menambah 1.15 rad)
θ3 = θ2 + q_wrist                 (arah gripper)

ee = shoulder(0, 0.475) + L1·u(θ1+φ0) + L2·u(θ2) + 0.08·u(θ3),   u(θ) = (sin θ, cos θ)
L1 = hypot(0.0175, 0.275) ≈ 0.2756,  φ0 = atan2(-0.0175, 0.275),  L2 = 0.2
```

Inverse kinematics (`src/kinematics/arm_kinematics.cpp`), target `(x, y, z)` + arah gripper (default menghadap ke bawah):

1. `hip = atan2(y, x)`, `r = hypot(x, y)`
2. Mundur dari ujung gripper sejauh 0.08 m ke titik wrist, sehingga sisa masalahnya IK 2-link
3. Hukum cosinus: `cos δ = (d² − L1² − L2²) / (2·L1·L2)`, lalu `shoulder`, `elbow` dari geometri
4. `wrist = tool_pitch − θ2` agar arah gripper tetap

Hanya satu solusi elbow yang berada dalam limit joint pada robot ini. FK dan IK sudah diverifikasi terhadap MuJoCo
(selisih posisi < 1e-6 m).

Lintasan: segitiga sama sisi (default bidang `xz`, pusat `(0.33, 0, 0.30)` relatif base, jari-jari `0.08` m),
titik bergerak linear antar sudut dalam satu `period` (default 6 s). Saat start, node memeriksa seluruh keliling:
bila ada titik di luar workspace/limit joint, error dicetak di awal.

**Kompensasi gravitasi.** Aktuator Modul 2 adalah kontrol posisi murni (`kp`), sehingga lengan "turun" akibat beratnya
(error ujung gripper ±2 cm). `control_gravity_compensation()` (callback `mjcb_control`) meniadakan gaya gravitasi,
sehingga error turun menjadi ±1.3 mm. Dapat dimatikan dengan `gravity_comp:=false`.

## Perubahan terhadap kode Modul 2

| File | Perubahan |
|---|---|
| `models/arm_final.xml` | Ditambah 1 baris: `<site name="ee">` di ujung gripper (titik acuan end-effector) |
| `sim/`, `callback/` | Tidak diubah |
| `control/` | Ditambah `control_gravity_compensation()` |
| `main.cpp` → `mujoco_sim_node.cpp` | Ditambah init ROS, `RosBridge`, `ros::spinOnce()`, dan `sim_advance()` (real-time) menggantikan `mj_step()` per frame |
| Baru | `ros_bridge/`, `kinematics/`, `triangle_trajectory_node.cpp`, `sync_node.cpp`, launch, `dual_arm.xml` |

`models/dual_arm.xml` dibuat oleh `scripts/make_dual_arm.py` dari `arm_final.xml` (dua salinan, nama diberi prefix
`arm1_`/`arm2_`, base di x = −0.4 dan +0.4).

## Prasyarat dan build

- Ubuntu 20.04 + ROS Noetic, MuJoCo (versi yang sama dengan Modul 2), `sudo apt install libglfw3-dev`
- Letakkan MuJoCo di `~/.mujoco/mujoco-x.y.z` atau `export MUJOCO_DIR=/path/ke/mujoco-x.y.z`

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone <URL-REPO-KAMU> kpp_ros_mujoco
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

## Menjalankan

```bash
roslaunch kpp_ros_mujoco single_arm.launch                      # wajib: 1 robot + segitiga
roslaunch kpp_ros_mujoco single_arm.launch plane:=xy            # segitiga datar (hip ikut bergerak)
roslaunch kpp_ros_mujoco dual_arm.launch                        # bonus: 2 robot sinkron
roslaunch kpp_ros_mujoco dual_arm.launch arm2_phase_offset:=0.5 # fase berlawanan
# Tanpa jendela (mis. via SSH): tambahkan headless:=true
```

Viewer: mouse untuk kamera, spasi untuk reset. Tombol keyboard Modul 2 (Q/A, W/S, E/D, R/F, T/G) tetap ada
untuk robot pertama; hentikan trajectory lebih dulu (`enable false`) karena perintah ROS menimpa `ctrl` terus-menerus.

## Pengujian dengan ROS CLI

```bash
rostopic list
rostopic info /arm1/joint_cmd
rostopic echo /arm1/ee_position
rostopic echo /arm1/ee_target
rostopic echo -n 1 /arm1/joint_states
rostopic hz /arm1/joint_states            # sekitar 60 Hz (mengikuti frame rate viewer; 100 Hz saat headless)

# Kontrol manual: pause trajectory, kirim perintah sendiri, lalu lanjutkan
rostopic pub -1 /arm1/enable std_msgs/Bool "data: false"
rostopic pub -1 /arm1/joint_cmd std_msgs/Float64MultiArray "data: [0.5, 0.2, 0.3, 0.3]"
rostopic pub -1 /arm1/joint_cmd std_msgs/Float64MultiArray "data: [0, 0.2, 0.3, 0.3, -0.4, -0.4]"  # + gripper
rostopic pub -1 /arm1/enable std_msgs/Bool "data: true"

# Bonus: kontrol terpisah. Hentikan arm2 saja, arm1 tetap bergerak
rostopic pub -1 /arm2/enable std_msgs/Bool "data: false"
```

## Bukti simulasi

<!-- Tambahkan screenshot / GIF / link video + output rostopic di sini -->
- `docs/single_arm.png`
- `docs/dual_arm.gif`
- Output `rostopic list`, `rostopic echo /arm1/ee_position`

## Parameter

| Node | Parameter | Default | Arti |
|---|---|---|---|
| `mujoco_sim_node` | `model_path` | wajib | file XML MuJoCo |
| | `robots` | `[arm1]` | daftar nama robot (namespace) |
| | `use_prefix` | `false` | `true` bila nama di XML berawalan `<robot>_` (multi-robot) |
| | `gravity_comp` | `true` | kompensasi gravitasi |
| | `headless` | `false` | tanpa viewer |
| | `cam_distance` | `1.5` | jarak kamera |
| `triangle_trajectory_node` | `period` | `6.0` | detik per putaran |
| | `size`, `center_x/y/z` | `0.08`, `0.33/0/0.30` | ukuran dan pusat segitiga (frame base) |
| | `plane` | `xz` | bidang segitiga: `xz`, `xy`, `yz` |
| | `tool_pitch` | `π` | arah gripper (π = ke bawah) |
| | `phase_offset` | `0.0` | geser fase 0..1 |
| | `use_external_phase` | `false` | ikut `/sync/phase` |

## Struktur repo

```
ROS-1/                        (package catkin: kpp_ros_mujoco)
├── CMakeLists.txt, package.xml
├── docker/   Dockerfile
├── include/  sim.hpp  callback.hpp  control.hpp  ros_bridge.hpp  arm_kinematics.hpp
├── src/      sim.cpp  callback.cpp  control.cpp  ros_bridge.cpp  arm_kinematics.cpp
│             mujoco_sim_node.cpp  triangle_trajectory_node.cpp  sync_node.cpp
├── launch/   single_arm.launch  dual_arm.launch
├── models/   arm_final.xml  dual_arm.xml  arm.urdf
├── scripts/  arm_dual_make.py
├── docs/     screenshot dan rekaman simulasi
└── README.md
'''