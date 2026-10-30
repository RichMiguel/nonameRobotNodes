# Topik: /map dan /map_metadata

Tipe Pesan:  
- `/map`: `nav_msgs/msg/OccupancyGrid`  
- `/map_metadata`: `nav_msgs/msg/MapMetaData`  
Penerbit (Publisher): `map_server`  
Penerima (Subscribers): `amcl`, `global_costmap`, RViz2  
Frekuensi: Transient Local (Dipublikasikan saat startup dan setiap peta baru dimuat)  
QoS: Durability Transient Local, Reliability Reliable

---

## 1. Deskripsi & Peran
Topik `/map` berisi matriks grid probabilitas keberadaan dinding atau ruangan lantai rumah sakit (*Occupancy Grid 2D*). Peta ini dijadikan acuan dasar bagi `global_costmap` untuk merencanakan rute jalan dan bagi `amcl` untuk mencocokkan pantulan laser LiDAR.

---

## 2. Struktur Pesan (`nav_msgs/msg/OccupancyGrid`)
```yaml
std_msgs/Header header:
  builtin_interfaces/Time stamp
  string frame_id                  # 'map'
nav_msgs/MapMetaData info:
  builtin_interfaces/Time map_load_time
  float32 resolution               # Ukuran per pixel: 0.05 meter (5 cm)
  uint32 width                     # Lebar dalam sel/pixel
  uint32 height                    # Panjang dalam sel/pixel
  geometry_msgs/Pose origin        # Koordinat pojok kiri bawah peta (x, y, yaw)
int8[] data                        # Array 1D (panjang width * height)
```

### Nilai Sel Array (`int8[] data`):
- **`0`**: Ruang bebas (*Free Space* / lantai kosong yang bisa dilalui robot).
- **`100`**: Rintangan padat (*Occupied Space* / dinding permanen atau pilar bangunan).
- **`-1`**: Area yang belum diketahui (*Unknown Space* / bagian luar gedung atau ruangan terkunci).
