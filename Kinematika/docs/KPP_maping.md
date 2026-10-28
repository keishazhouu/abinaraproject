# Abinara — KPP Level 1 (ROS Noetic + MuJoCo)

## Apa yang berubah
- Simulasi MuJoCo dan `ros_bridge.cpp` dipertahankan dari project awal.
- `triangle_trajectory_node` tidak dijalankan pada launch Level 1.
- `ik_command_node` menghitung IK untuk satu target posisi Cartesian, lalu mengirim empat sudut joint ke `/arm1/joint_cmd`.
- Kalkulasi dibungkus kelas `InverseKinematics` di `include/inverse_kinematics.hpp` dan `src/inverse_kinematics.cpp`.
- Tidak ada lintasan segitiga atau fitur gambar/tulisan.

## Build
Letakkan folder `kpp_ros_mujoco` di `~/catkin_ws/src/`, lalu:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Pastikan `MUJOCO_DIR` sudah sesuai dengan instalasi MuJoCo yang dipakai project awal.

## Run

```bash
roslaunch kpp_ros_mujoco single_arm.launch
```

Target bisa diubah lewat argumen launch, misalnya:

```bash
roslaunch kpp_ros_mujoco single_arm.launch target_x:=0.30 target_y:=0.0 target_z:=0.30 tool_pitch:=0.0
```

`target_x`, `target_y`, dan `target_z` menggunakan meter; `tool_pitch` menggunakan radian. Node hanya menerbitkan command jika IK menandai target reachable dan sudut memenuhi limit joint.

## Alur demo
1. Tunjukkan target Cartesian (x, y, z) yang dipilih.
2. Jelaskan bahwa IK mengubah target tersebut menjadi empat sudut joint.
3. Tunjukkan nilai sudut dalam derajat di log ROS. Nilai yang dikirim ke ROS adalah radian.
4. Tunjukkan `/arm1/joint_cmd` dan pergerakan robot di MuJoCo.

## Catatan validasi
IK ini memakai fungsi geometri dari `arm_kinematics.cpp` project awal. Kompilasi node dan simulasi belum bisa diverifikasi di lingkungan ROS/MuJoCo milik pengguna dari paket ini saja. Sebelum demo final, bandingkan posisi target dengan `/arm1/ee_position`, dan kalibrasikan `tool_pitch` serta target terhadap frame yang dipakai model MuJoCo. Jangan menganggap hasil IK sudah cocok secara fisik hanya karena build berhasil.