# Topik: /amcl_pose dan /particlecloud

Tipe Pesan:  
- `/amcl_pose`: `geometry_msgs/msg/PoseWithCovarianceStamped`  
- `/particlecloud`: `nav2_msgs/msg/ParticleCloud`  
Penerbit (Publisher): `amcl`  
Penerima (Subscribers): RViz2, `fsm_node`, modul diagnostik  
Frekuensi Tipikal: $2.0\text{–}5.0\text{ Hz}$  
QoS: Volatile, Reliable / Best Effort  

---

## 1. Deskripsi & Peran
1. **`/amcl_pose`**: Menampilkan estimasi terbaik posisi dan orientasi robot pada peta global `map`. Matriks kovariansi menggambarkan seberapa yakin AMCL terhadap posisinya (nilai kovariansi mengecil saat robot mengenali dinding dengan jelas, dan membesar saat berada di ruang kosong terbuka).
2. **`/particlecloud`**: Berisi seluruh sebaran partikel hipotesis posisi saat ini. Setiap partikel memiliki koordinat $(x, y)$ dan orientasi yaw sendiri serta bobot (*weight*). Saat divisualisasikan di RViz2, kumpulan panah merah ini menggambarkan proses konvergensi filter probabilitas.

---

## 2. Struktur Pesan `/particlecloud` (`nav2_msgs/msg/ParticleCloud`)
```yaml
std_msgs/Header header:
  builtin_interfaces/Time stamp
  string frame_id                      # 'map'
nav2_msgs/Particle[] particles:
  geometry_msgs/Pose pose              # Hipotesis posisi partikel
  float64 weight                       # Bobot probabilitas kecocokan laser ke peta
```
