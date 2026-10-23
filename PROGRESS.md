# PROGRESS

Her oturum sonunda en üste yeni bir kayıt eklenir.

---

## 2026-10-23 — Yön değişikliği ve teknoloji kararları

**Yapıldı**
- Proje yeniden çerçevelendi: arama-kurtarma çıktı, **otonom semantik haritalama** (insan + araç katmanı) oldu. Repo adı: `uav-semantic-mapping`.
- Kapsam "A + B" olarak kesinleşti: ölçüm odaklı çekirdek + canlı tespit/konum (TensorRT, Rerun). C++ konumlandırma ve ByteTrack 20 Kasım sonrasına.
- Model incelendi (dosya çalıştırılmadan, metadata okundu): standart `yolo11l.yaml`, imgsz 1280, 10 sınıf (UAP, INSAN, MOTOR, ARABA, KAMYON, OTOBUS, UAI, MAKINE, GEMI, TREN), Ultralytics 8.3.155, nadir drone görüntüleriyle eğitilmiş. "Değiştirilmiş mimari" iddiası kaldırıldı.
- Model `models/teknofest_yolo11l_nadir.pt` olarak taşındı; `.gitignore` oluşturuldu (ağırlıklar, `.venv`, veri, kayıtlar, COLMAP çıktıları, ROS build).
- CLAUDE.md bu kararlara göre yeniden yazıldı.

**Kararlar**
1. Tarama nadir (−90°), gimbal dengeleme yapar; eğik açı deneyi ayrı ve "zaman kalırsa".
2. İrtifa geçici olarak h = 30 m (1920×1080, HFOV 60° varsayımıyla GSD ≈ 1,8 cm/px, insan ≈ 25 px; ağ girişinde ≈ 17 px). Hafta 2'de irtifa taramasıyla kesinleşecek.
3. Modelin eğitim verisi belgelenmemiş kabul edildi: VisDrone test-dev'de ölçülür, train/test-dev farkı ezber işareti olarak raporlanır.
4. Tespit: offline (resmi sayılar) + canlı (TensorRT, demo). Canlıda kutular hesaplandıkları kareye çizilir.
5. Demo videosu karma: simülasyon ana bölüm + "real-world" etiketli gerçek veri bölümü (drone yok → telemetrili açık veri seti, aday AU-AIR).
6. Teknolojiler: rosbag2 + MCAP, Rerun, pycocotools (AP) + kendi konum metrikleri, uv, AGPL-3.0.
7. Konumlandırma: EKF2 pozu + ölçülen gimbal açısı + düz zemin (z = 0) + sınıf yüksekliği düzeltmesi. Gazebo gerçek pozları sadece ölçümde kullanılır.

**Ölçüldü**
- (henüz yok — sadece hesap: 30 m'de 3° gimbal hatası ≈ 1,6 m, 5° ≈ 2,6 m kayma; ayakta insan görüntü kenarında nesne yüksekliği yüzünden ≈ 1 m kayma)

**Sıradaki adım**
- Hafta 1: Gazebo Harmonic + PX4 SITL + Micro XRCE-DDS kurulumu; `x500_gimbal` ile ilk kalkış/iniş

**Açık kararlar**
- PX4 sürüm etiketi (kurulumda)
- Değerlendirmede sınıf eşlemesi (MOTOR araç sayılacak mı?) ve IoU eşiği (hafta 2, ölçümden önce)
- Gerçek dünya demo veri seti seçimi (hafta 2)

---

## 2026-10-22 — Proje kararı

**Yapıldı**
- Proje seçildi: arama-kurtarma drone'u (tespit + konumlandırma), adı `uav-sar-geolocation`
- Kararlar: PX4, gimbal kamera, köy/kasaba dünyası, insan + araç, HERIDAL + VisDrone, şerit + olasılık haritası (olasılık haritası ilk ertelenecek), C++ modülü 20 Kasım'dan sonra
- Güncelleme: 2D web haritası çıkarıldı; yerine görev sonu RTL + COLMAP ile 3B harita + tespitlerin 3B haritada işaretlenmesi + uçuş sonu raporu geldi. Tespit modeli: Oğuz'un önceki projesindeki kendi YOLO11 modeli.
- Ortam kontrol edildi: Ubuntu 24.04, ROS 2 Jazzy kurulu, GTX 1650 (sürücü 590, CUDA 13.1), ~50 GB boş disk

**Ölçüldü**
- (henüz yok)

**Sıradaki adım**
- Hafta 1: Gazebo Harmonic + PX4 SITL + Micro XRCE-DDS kurulumu; ilk kalkış/iniş

**Açık kararlar**
- Repo lisansı (Ultralytics AGPL-3.0 olduğu için)
- HERIDAL lisansının kontrolü
