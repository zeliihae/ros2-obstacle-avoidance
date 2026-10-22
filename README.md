# ROS 2 Otonom Engelden Kaçınma Sistemi 🚗🤖

Bu proje, otonom araç sistemlerinin temelini oluşturan **Algılama-Karar-Kontrol (Perception-Planning-Control)** döngüsünü kavramak ve ROS 2, C++, GitHub entegrasyonlarını pratikte uygulamak amacıyla geliştirilmektedir. 

Gazebo simülasyon ortamında, SDF ile tanımlanmış diferansiyel sürüşlü ve LiDAR sensörlü özel bir robot modeli kullanılmaktadır. Proje, otonom araçlardaki AEB (Acil Fren) ve engelden kaçınma mantığının simüle edilmiş bir versiyonudur.

## 🏗️ Sistem Mimarisi

Sistem, ROS 2 üzerinde C++ ile geliştirilen 3 temel düğümden (node) oluşmaktadır:

1. **`perception_node` (Algılama):** `/scan` topic’ine abone olur, LiDAR verisinden önündeki en yakın mesafeyi hesaplar ve `/obstacle_distance` olarak yayınlar.
2. **`decision_node` (Karar):** Mesafeye göre durum belirler (SERBEST / YAVAŞLA / DUR / KAÇIN). Basit bir state machine olarak görev yapar. *(Geliştirme aşamasında)*
3. **`control_node` (Kontrol):** Kararı `/cmd_vel` komutuna çevirir (hız ve dönüş). *(Geliştirme aşamasında)*

**Veri Akışı:**
`/scan` ➡️ `perception_node` ➡️ `/obstacle_distance` ➡️ `decision_node` ➡️ `/state` ➡️ `control_node` ➡️ `/cmd_vel`

---

## ✅ Tamamlanan Aşamalar (Aşama 1 - Algılama)

- **Versiyon Kontrolü:** Proje için GitHub üzerinde repo oluşturuldu ve geliştirme süreci adım adım commit edilerek kayıt altına alındı.
- **Çalışma Alanı ve Paket Kurulumu:** `ros2-obstacle-avoidance` çalışma alanı oluşturuldu. Gerekli bağımlılıkları (`rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`) içeren paket `ament_cmake` ile derlendi (`colcon build`).
- **Simülasyon Entegrasyonu:** Özel robot modeli ve engelleri barındıran Gazebo dünyası ayağa kaldırıldı. Simülasyon için launch dosyası yazılarak Gazebo ve `ros_gz_bridge` entegrasyonu tek komuta indirgendi.
- **Algılama Düğümü Geliştirildi:** Ön görüş açısındaki (+/- 30.0 derece) engelleri tespit etmek için filtreleme mantığı kurgulandı. Elde edilen anlık minimum engel mesafesi `/obstacle_distance` topic’ine `std_msgs/Float32` tipinde yayınlandı.
- **Mühendislik Kararı (-inf Yönetimi):** Engel, LiDAR sensörünün minimum menzilinden (0.12 m) daha yakınsa sensörden dönen `-inf` değeri kod içerisinde yakalanarak "çok yakın engel" olarak güvenli bir şekilde işlendi.

---

## 🧪 Sistem Doğrulaması ve Testler

Simülasyon ortamında gerçekleştirilen test senaryoları:

* **Engele Yaklaşım:** Robot statik bir engele doğru ilerlerken, `perception_node` engel mesafesini gerçek zamanlı olarak (örn: 1.64m ➔ 1.02m) doğru ölçüp yayınladı.
* **Statik Ölçüm Doğrulaması:** Başlangıç konumunda hareketsizken kutuya olan mesafe 3.35m olarak ölçüldü. *(Hesaplama: Kutunun merkezden uzaklığı (3.5m) - LiDAR'ın robot merkezinden konumu (0.15m) = 3.35m)*.
* **Serbest Alan:** Engeller algılama menzilinin dışında olduğunda, LiDAR verisinin maksimum sınır değeri olan `8.0` istikrarlı bir şekilde okundu.

---

## 💻 Çalışma Ortamı ve Terminal Düzeni

Geliştirme aşamasında sistemin anlık izlenebilmesi için aşağıdaki yapı kullanılmıştır (İlerleyen haftalarda tüm bu süreçler tek bir launch dosyasına entegre edilecektir):

1. **Terminal 1 (Launch):** Gazebo simülasyonu ve köprü başlatılır.
   ```bash
   ros2 launch obstacle_avoidance simulation.launch.py
   ```
2. **Terminal 2 (Perception):** Algılama düğümü çalıştırılır.
   ```bash
   ros2 run obstacle_avoidance perception_node
   ```
3. **Terminal 3 (Topic Echo):** Yayınlanan mesafe doğrulanır.
   ```bash
   ros2 topic echo /obstacle_distance
   ```
4. **Terminal 4 (Manuel Kontrol):** Robota test amaçlı `/cmd_vel` yayınları yapılır.

---

## 🛠️ Karşılaşılan Sorunlar ve Çözümler

* **WSL Performansı:** Windows Subsystem for Linux (WSL) üzerinde çalışırken LiDAR frekansının yaklaşık 6 Hz seviyelerinde çalıştığı gözlemlendi. Sistem karar mekanizmaları bu yenileme hızına uygun tepki verecek şekilde kalibre edilecek.
* **SDF Uyarıları:** Simülasyon başlatılırken konsolda beliren bazı SDF etiketleme uyarılarının, fizik motorunu etkilemeyen zararsız mesajlar olduğu tespit edilip göz ardı edildi.
