# PROGRESS

Her oturum sonunda en üste yeni bir kayıt eklenir.

---

## 2026-10-26 — PX4 ↔ ROS 2 köprüsü, ilk kalkış, proje araç dosyası

**Yapıldı**
- Micro XRCE-DDS Agent v2.4.3 kaynaktan derlendi (`~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`); `sudo make install` gerekmedi.
- `px4_msgs` (`release/1.17`) `~/px4_ws`'de derlendi (3 dk 21 s). 235 güncel mesaj PX4 v1.17.0 tanımlarıyla birebir aynı (eksik 9 tanesi yalnızca eski `*V0` sürümleri).
- Köprü uçtan uca çalışıyor: `ros2 topic list`'te `/fmu/in/*` ve `/fmu/out/*`; `vehicle_local_position_v1` okundu.
- İlk kalkış/iniş (`commander takeoff`/`land`, pxh konsolundan).
- Proje araç dosyası `4900_gz_usm_x500_gimbal` + `setup/px4_overlay.sh` (yamalar + link, idempotent; iki kez çalıştırılarak doğrulandı) + yama `0002` (araç dosyasını PX4 listesine kaydeder). Yeni make hedefi: `gz_usm_x500_gimbal`.

**Ölçüldü / gözlendi**
- Yerde: x/y/z ≈ 0,008 / 0,015 / 0,063 m; EKF belirsizliği eph 0,15 m, epv 0,17 m; heading 1,676 rad (~96°).
- Kalkış sonrası: z = −2,48 m (NED, z aşağı → h = 2,48 m), x/y sapması < 5 cm.

**Sorunlar ve çözümler**
- "No connection to the GCS": `NAV_DLL_ACT` x500'de 2. `PX4_PARAM_NAV_DLL_ACT=0` işe yaramadı — kök neden: `param set` değer varsayılana (0) eşitse yazmıyor, sonra araç dosyasının `set-default 2`'si kazanıyor (`src/systemcmds/param/param.cpp` do_set). Çözüm: kendi araç dosyamızda `param set-default NAV_DLL_ACT 0`. Doğrulandı: temiz parametrelerle 0, kalkış engelsiz.
- İlk denemede model bulunamadı (`usm_x500_gimbal/model.sdf`): PX4 model adını make hedefinden türetiyor; araç dosyasında `PX4_SIM_MODEL=gz_x500_gimbal` koşulsuz atandı.
- Terminal penceresini kapatmak PX4/Gazebo'yu kapatmıyor; elle sonlandırıldı.

**Sıradaki adım**
- `ros/usm_mission` paketi + `offboard_hover` node'u (Claude yazar, açıklayarak): arm → offboard → 5 m → 10 s bekle → in. Doğrulama: pxh'ye komut yazmadan kalkış/iniş.

**Açık kararlar**
- Offboard arayüzü teknolojisi (bkz. sıradaki konuşma)

---

## 2026-10-23 (öğleden sonra) — Kurulum: NVIDIA sürücüsü, PX4 v1.17.0, Gazebo Harmonic

**Yapıldı**
- NVIDIA sürücüsü onarıldı: otomatik güncelleme çekirdeği 7.0.0-38'e taşımış, NVIDIA modülü sadece 6.17 için vardı → `nvidia-driver-595-open` + `linux-modules-nvidia-595-open-generic-hwe-24.04` (Ubuntu 590 → 595 geçişi yapmış). Sonuç: sürücü 595.99.02, CUDA 13.2, PyTorch GPU'yu görüyor.
- `~/.bashrc` temizlendi (var olmayan `~/ros2_ws` ve eski TurtleBot satırları). Yedek: `~/.bashrc.bak-2026-10-23`.
- PX4 **v1.17.0** alt modüllerle `~/PX4-Autopilot` (2,8 GB). Eşleşen `px4_msgs` dalı: `release/1.17`.
- PX4 Python bağımlılıkları ayrı venv'e: `~/px4-venv` (sistem Python'u temiz kaldı). `ubuntu.sh --no-nuttx` ile Gazebo Harmonic (OSRF, gz-sim 8.15) + ninja kuruldu.
- `make px4_sitl gz_x500_gimbal` çalışıyor: Gazebo açılıyor, gimbal'lı drone görünüyor, `pxh>` geliyor.
- **RAM sızıntısı bulundu ve giderildi:** PX4'ün `GstCameraSystem` eklentisi (kamera → H.264 → UDP video akışı). Kök neden: (1) Ubuntu 24.04 GStreamer'ındaki `nvh264enc` sürücü 595 ile başlamıyor ("Selected preset not supported"); (2) eklentinin `appsrc` girişinde boyut sınırı/hata kontrolü yok → her kare RAM'de birikiyor. Eklenti `server.config`'te kapatıldı; yama: `setup/px4_patches/0001-disable-gst-camera-plugin.patch`.

**Ölçüldü**
- Gazebo (arayüzlü, ilk çalıştırma): RTF %99,45; VRAM 685 MiB (sunucu 338 + arayüz 320 + Xorg 17); GPU %38.
- Sızıntı testleri (arayüzsüz, gz sunucu RSS):

  | Test | Kamera | GStreamer eklentisi | Sonuç |
  |---|---|---|---|
  | A | var | açık | +79 MB/s (1280×720×3 B × 30 fps ≈ 83 MB/s ile uyumlu) |
  | B (`gz_x500`) | yok | açık | 0,22 GB sabit |
  | C (NVIDIA yönlendirmesiz) | var | açık | +79 MB/s |
  | D | var | kapalı | 1,18 GB sabit |
  | E (kalıcı yama ile) | var | kapalı | 1,18 GB sabit, 40 s |
- `gst-launch` testi: `nvh264enc` başarısız, `x264enc` çalışıyor.

**Sorunlar / notlar**
- Makinede iki Gazebo Harmonic var: ROS vendor (8.10, ROS yüklü kabukta) ve OSRF (8.15). PX4 ROS yüklü kabukta derlenip çalıştırıldığı için tutarlı olarak 8.10 kullanıyor.
- PX4 gimbal kamerası varsayılanı: 1280×720, 30 Hz, HFOV 2,0 rad (~115°) — bizim 60° varsayımımızdan çok geniş; kendi kamera ayarımız yapılacak.
- Preflight "No connection to the GCS" → arm edilemez; uçuştan önce QGC ya da SITL parametresiyle çözülecek.

**Sıradaki adım**
- Micro XRCE-DDS Agent + `~/px4_ws` içinde `px4_msgs` (`release/1.17`) → `ros2 topic list`'te `/fmu/out/...` görünmesi
- GCS kontrolünün çözülmesi ve `commander takeoff` ile ilk kalkış/iniş

**Takvim kararı**
- Hedef 20 Kasım, yedek 1 Aralık. Ara kontroller 4 / 11 / 14 Kasım (14 Kasım özellik dondurma). Kesme sırası ve video planı CLAUDE.md §4'te.
- Hafta 1 büyük ihtimalle hafta 2'ye taşacak (bugün sürücü + RAM sızıntısına zaman gitti).

**Açık kararlar**
- GCS kontrolü: QGroundControl mı, SITL parametresi mi?
- Gerekirse hafta sonu tampon saatleri (Oğuz'un cevabı bekleniyor)
- PX4 `GstCameraSystem` hatasını upstream'e issue olarak bildirmek (isteğe bağlı)

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
