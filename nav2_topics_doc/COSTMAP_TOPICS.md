# Topik: Costmap Grids dan Published Footprints

Topik yang Tercakup:
- `local_costmap/costmap_raw` & `global_costmap/costmap_raw`
- `local_costmap/published_footprint` & `global_costmap/published_footprint`  
Tipe Pesan:  
- `costmap_raw`: `nav2_msgs/msg/Costmap`  
- `published_footprint`: `geometry_msgs/msg/PolygonStamped`  
Penerbit (Publishers): `local_costmap`, `global_costmap`  
Penerima (Subscribers): `controller_server`, `behavior_server`, RViz2  
Frekuensi: $1.0\text{–}4.0\text{ Hz}$  

---

## 1. Topik `costmap_raw`
Membawa data grid biaya (*cost grid*) dari lapisan rintangan dan inflasi:
- **`local_costmap/costmap_raw`**: Window berukuran $3\text{m} \times 3\text{m}$ (resolusi $5\text{ cm}$) yang mengikuti robot. Digunakan oleh `controller_server` untuk deteksi tabrakan instan ($< 0.45\text{ detik}$) dan deselerasi kedekatan rintangan.
- **`global_costmap/costmap_raw`**: Seluruh area denah rumah sakit yang mencakup penalti biaya dinding untuk memandu robot melaju di tengah lorong koridor.

### Nilai Biaya Sel (Cost Values 0–255):
- `0`: Bebas tanpa biaya (*Free space*).
- `1` s/d `252`: Biaya inflasi gradien (*Inscribed buffer* / zona penalti kedekatan).
- `253`: Zona tabrakan mutlak poligon bodi robot (*Lethal obstacle*).
- `254`: Rintangan terdeteksi laser LiDAR.
- `255`: Area yang belum diketahui (*Unknown space*).

---

## 2. Topik `published_footprint`
Menampilkan poligon bentuk fisik robot $44\text{ cm} \times 44\text{ cm}$ (radius sasis $0.24\text{ m}$) yang diproyeksikan ke posisi robot saat ini di RViz2:
```yaml
geometry_msgs/Polygon polygon:
  geometry_msgs/Point32[] points:
    - {x: 0.22,  y: 0.22}   # Front-Left
    - {x: 0.22,  y: -0.22}  # Front-Right
    - {x: -0.22, y: -0.22}  # Rear-Right
    - {x: -0.22, y: 0.22}   # Rear-Left
```
