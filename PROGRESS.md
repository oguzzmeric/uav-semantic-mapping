# PROGRESS

Her oturum sonunda en üste yeni bir kayıt eklenir.

---

## 2026-10-27 — Kamera köprüsü, saat eşitleme, ilk kayıtlı görev

**Yapıldı**
- Kamera `ros_gz_bridge` ile ROS'a alındı: `/camera/image`, `/camera/camera_info`; ayrıca `/clock` ve Gazebo model pozları (`/gz/dynamic_pose`).
- `ros/usm_mission/launch/record.launch.py`: köprü + `ros2 bag record` (MCAP, kayıpsız `zstd_fast`), kayıt `data/bags/<tarih_saat>/` (gitignore'da). Argümanlar: `world`, `out`. Repo kökünden çalıştırılır.
- Araç dosyası: `UXRCE_DDS_SYNCT 0` → PX4 damgaları simülasyon saatinde (commit `faba8f2`).
- İlk kayıtlı görev uçuşu (Oğuz'un arayüzlü simülasyonu, 4 terminal).

**Ölçüldü**
- Kamera: `ros2 topic hz` 5,7 Hz gösterdi ama rosbag2 (C++) 11,57 s'de 115 kare = **9,94 Hz**. Darboğaz Python ölçüm aracı (6 MB/kare); geçen haftaki ~6 Hz ölçümü de büyük ihtimalle aynı sebepten. Render NVIDIA'da (400 MiB, %18), RTF 0,99.
- Ham kayıt 59 MB/s (5,94 MB/kare = 1920×1080×3). Boş dünyada zstd 683 MB → 1,8 MB (~380×) ve JPEG q95 kare başına 48 KB — **anlamsız**: kare gökyüzü + gri zemin, 43 farklı renk. Biçim kararı dokulu dünyada ölçülerek verilecek.
- Saatler: varsayılan ayarda kamera 250,9 s (sim), PX4 1 793 108 608 s (Unix) → eşlenemez. Kök neden: `UXRCE_DDS_SYNCT=1` PX4 damgalarını Agent'ın duvar saatine çeviriyor; SITL'de PX4 saati zaten Gazebo saati (`GZBridge::clockCallback`). Kapatınca: kamera 28,700 / gimbal 28,704 / konum 28,740 s; varışta gz saati − PX4 damgası medyan 4 ms, maks 8 ms (197 örnek). Gelen setpoint damgaları `min(damga, şimdi)` ile kırpıldığı için offboard bozulmadı (hover testi).
- Kayıtlı görev (`data/bags/20261027_170055`, 421 s, 75 MB): 4185 kare, sim saatinde **10,00 Hz**, en büyük kare aralığı 100 ms (kare kaybı yok). Uçuştaki 1878 kareye en yakın örnek: EKF2 konumu maks **4 ms**, araç yönelimi maks 4 ms, gimbal medyan 4 / maks 24 ms.
- Görev karnesi uçuş 3 ile neredeyse aynı (tekrarlanabilir): gimbal şeritte 0,61° / 3,38°, yan sapma 0,07 / 0,93 m, tarama 171,4 s.

**Sorunlar**
- `/gz/dynamic_pose` kullanılamaz: köprünün `Pose_V → TFMessage` dönüşümü model adlarını ve damgaları düşürüyor (ad boş, damga 0; sadece sıra kalıyor). Gerçek pozlar için damgalı başka bir kaynak gerekli.
- Kendi başlatma/kapatma betiklerim: `set -u` ROS kurulum betiğini bozdu; Agent `INT` ile kapanmıyor (`TERM` gerekli); kapatma betiği bir kez yarıda kaldı, süreçler PID ile kapatıldı (önce `HEADLESS=1` ile benim olduğu doğrulandı).
- `data/bags/20261027_165543` (5,1 MB): kayıt launch'ının kısa ilk denemesi, kullanılmayacak.

**Ek saat: gerçek poz kaynağı ve ilk EKF2 ↔ gerçek konum ölçümü**
- Karar: `OdometryPublisher` (seçenek a). `usm_x500_gimbal` modeline eklendi (dünya pozu ENU + hız, 50 Hz, sim saati); kayıtta `/gz/dynamic_pose` yerine `/gz/odometry` (commit `741d5df`).
- Arayüzsüz görev kaydı (255 s, 31 MB, 2529 kare): odometri 12 643 mesaj, 49,6 Hz; yerde konum (0, 0, −0,013), çerçeveler `world` → `base_link`.
- İlk karşılaştırma (scratchpad betiği; ENU → NED, gerçek poz EKF2 `timestamp_sample` anına doğrusal enterpolasyonla; tarama = OFFBOARD ve z ±0,5 m içinde, 8601 örnek):
  - Yerde (ilk 3 s): yatay medyan 0,7 cm; aşağı −0,046 m; **yaw +10°** (başlangıçta yön oturmamış).
  - Taramada: ortalama K +0,017 / D +0,033 / aşağı −0,059 m (sistematik ≈ 0), ama std K 0,43 / D 0,18 m, yatay **medyan 0,50 m**, p95 0,67 m.
  - Zaman kayması taraması: gerçek poz +100 ms kaydırılınca yatay RMS **0,464 → 0,093 m** (−300…+300 ms, 20 ms adım). EKF2 gerçeğin ~100 ms önünde.
  - Muhtemel kök neden: `EKF2_GPS_DELAY` varsayılanı 110 ms (`params_gnss.yaml`); PX4'ün kendi simülasyon araçları bunu düzeltiyor (SIH 0, gazebo-classic 10), gz araçları (4001/4019) düzeltmiyor; simüle GPS gecikmesiz.
- Not: görev node'u duvar saatini kullanıyor → kayıttaki `/rosout` damgaları sim saatiyle uyuşmuyor (ölçümde pencere PX4 durumundan alındı). İleride `use_sim_time`.

- **Doğrulandı:** araç dosyasına `EKF2_GPS_DELAY 0` (tek değişiklik, commit `5f0088c`), aynı uçuş + aynı betik (8746 örnek): yatay medyan 0,50 → **0,075 m**, p95/maks 0,67/0,73 → 0,14/0,24 m, kuzey std 0,43 → 0,038 m, en iyi kayma +100 → **0 ms**, kaymasız RMS 0,464 → 0,083 m. Görev karnesi değişmedi (gimbal şeritte 0,64° / 2,80°, yan sapma 0,07 m, tarama 174,6 s).
- Kalan gözlemler: taramada **yaw std 2,2°** (ort. +0,40°) — nadirde görüntü kenarında (~17 m) ~0,6 m'lik kayma demek; gerçek EKF2 hatası mı, gerçek-yaw çıkarma formülüm mü, ayrıca bakılacak. Yerde yaw farkı tekrar ~12° (pusula sapması olabilir; havada kayboluyor).

**Sıradaki adım**
- Yaw saçılmasını incelemek (2,2°): zamana göre çizip dönüşlerle ilişkisine bakmak; gerçek yaw'ı quaternion'dan doğru çıkardığımı doğrulamak.
- Karşılaştırma betiğini scratchpad'den repoya almak (değerlendirme aracı olarak).
- Dışa aktarma betiği: her kare için görüntü + en yakın EKF2 pozu + yönelim + gimbal + gerçek poz → dosyalar.
- Düz köy dünyası (dokulu zemin, insan + araç modelleri) → kayıt biçimi ölçümü (zstd vs JPEG).

**Açık kararlar**
- Kayıt biçimi (dokulu dünyada ölçümden sonra).

---

## 2026-10-26 (akşam) — Şerit rota, ilk görev uçuşu, gimbal sabitleme

**Yapıldı**
- `usm_core/coverage.py` tamamlandı: `ground_footprint`, `gsd`, `lane_spacing`, `photo_spacing` (Oğuz), `_lane_positions` (Oğuz), `lawnmower` (Oğuz'un isteğiyle Claude). Kenar kuralı varsayılanı (ii): `edge_margin = Gw/2`, ilk/son şerit alan sınırında.
- `tests/test_coverage.py`: 22 test yeşil (elle hesaplanmış değerler, testler koddan önce yazıldı). Float regresyon testi: `tan(45°)` = 0,9999999999999999 → `ceil(2.0000000000000004)` fazladan şerit ekliyordu; `ceil(x − 1e-9)` ile düzeltildi, test düzeltmesiz kırmızı / düzeltmeli yeşil doğrulandı.
- Python ortamı: `uv` + `.venv` (`--system-site-packages`), pytest; ROS pytest eklentileri `addopts` ile kapalı.
- ROS paketleri venv Python'uyla derleniyor: `.venv/bin/python -m colcon build --base-paths ros --symlink-install` → node'lar venv'i açmadan `usm_core`'u görüyor (temiz kabukta doğrulandı). CLAUDE.md'ye yazıldı.
- `survey_mission` node'u: WAIT → STREAM → ENGAGE → CLIMB → POINT_GIMBAL → SURVEY → RETURN (PX4 RTL) → DONE. Hareketli setpoint (konum + hız ileri beslemesi), yaw sabit 0, gimbal −90° (ölçülen açı < 2° olana kadar bekler), görev sonunda takip karnesi.
- `px4_io.py`: 7 parametreli `command()`, `gimbal_take_control()` (DO_GIMBAL_MANAGER_CONFIGURE), `gimbal_pitch_yaw()` (DO_GIMBAL_MANAGER_PITCHYAW, roll+pitch ufka kilitli), `position_setpoint(..., velocity)`.
- Araç dosyası: `RTL_RETURN_ALT 30` (60 m'ye tırmanmadan döner; yüksekte uçarken o irtifada döner — `rtl_params.c` doğrulandı).
- PX4 yaması `0003-gz-gimbal-roll-pitch-lock.patch`: Gazebo gimbal sürücüsü (`GZGimbal.cpp`) kilit bayraklarını yok sayıyordu (`//TODO handle device flags`); artık her döngüde araç roll/pitch'ini joint açısından çıkarıyor, döngü 5 Hz → 50 Hz.
- Hareketli setpoint trapez hız profiline geçti (`accel` 2 m/s²): `v = min(v_max, v + a·dt, √(2·a·kalan))`.

**Ölçüldü** (SITL, 60×60 m, h = 30 m, 5 m/s, %70/%80 örtüşme → 7 şerit, 10,0 m aralık, yol 586 m)

| | Uçuş 1 | Uçuş 2: gimbal yaması | Uçuş 3: + trapez profil |
|---|---|---|---|
| Gimbal off-nadir ort / maks (tüm tarama) | 8,61° / 43,1° | 1,86° / 17,0° | 0,92° / 5,81° |
| Gimbal off-nadir ort / maks (şeritte) | – | – | 0,61° / 3,41° |
| Yan sapma ort / maks | 0,27 / 2,70 m | 0,26 / 2,47 m | 0,06 / 0,92 m |
| Yükseklik hatası ort / maks | 0,02 / 0,19 m | 0,01 / 0,20 m | 0,01 / 0,05 m |
| Şerit hızı ort / maks | 5,28 / 9,32 m/s | 5,28 / 9,29 m/s | 4,64 / 6,13 m/s |
| Tarama süresi | 128,6 s (yol/hız 117,2) | 128,8 s | 171,3 s (profil 161,7) |
| Görev süresi (kalkış → disarm) | 190,5 s | 190,7 s | 230,8 s |

- Gimbal nadire geliş: ~1 s; havada dururken ölçülen sapma 0,95–1,63° (joint sarkması olabilir; konumlandırmada ölçülen açı kullanılacağı için sapma kaynağı değil).
- Ara deneme (`MNT_DO_STAB 1`): 8,61° / 43,12° — etkisiz.

**Sorunlar ve çözümler**
- `MNT_DO_STAB` hipotezi yanlıştı: bu parametre yalnızca AUX/MAVLink v1 çıkışında PX4'ün kendi telafisini açıyor; x500_gimbal'ın MAVLink v2 çıkışında PX4 sabitlemeyi cihaza bırakıyor, Gazebo sürücüsü ise yapmıyordu. AUX'a geçmek ölçülen gimbal açısının yayınını kesiyordu (`MNT_MODE_OUT == 2` şartı) → sürücü yamalandı.
- Köşelerde 9,3 m/s "yetişme koşusu": setpoint duruştan anında 5 m/s'ye geçiyordu → trapez profil.
- `pytest.approx` iç içe tuple listelerinde toleransı uygulamıyor → testlerde nokta nokta karşılaştırma (`assert_waypoints_close`).
- Hata (Claude): plan satırını görmek için node'u, açık simülasyonu kontrol etmeden 4 s çalıştırdı; node PX4'ü OFFBOARD'a aldı ve muhtemelen arm etti. Drone yerde ve disarm kaldı. Kural tekrar: node/betik çalıştırmadan önce PX4 açık mı kontrol edilir.

**Sıradaki adım**
- Hafta 1'in kalanı: kamera görüntüsünü `ros_gz_bridge` ile ROS'a almak, kare hızını `ros2 topic hz` ile ölçmek, rosbag2 (MCAP) kaydı (görüntü + EKF2 pozu + gimbal açısı + Gazebo gerçek pozları), dışa aktarma betiği. Ardından düz köy dünyası.
- `survey_mission.py`'yi birlikte okuma oturumu (Oğuz'un mülakatta anlatabilmesi için).

**Açık kararlar**
- Kayıt stratejisi (1080p ham ~62 MB/s @10 Hz: JPEG mi, çekim noktalarında fotoğraf mı?)
- Yama 0003'ü PX4'e upstream PR olarak göndermek (isteğe bağlı; GstCameraSystem issue'su ile birlikte)

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

- **Offboard kararı:** Python + `px4_msgs` offboard (şimdi); 20 Kasım sonrası PX4 ROS 2 Interface Library (C++) ile taşınacak. İniş/eve dönüş PX4'ün kendi modlarıyla (`NAV_LAND`, `NAV_RETURN_TO_LAUNCH`).
- `ros/usm_mission` paketi: `px4_io.py` (QoS, kalp atışı, setpoint, komutlar) + `offboard_hover.py` (durum makinesi: WAIT → STREAM → ENGAGE → CLIMB → HOVER → LAND → DONE). Repo kökünden `colcon build --base-paths ros --symlink-install`.
- **İlk kendi kodumuzla uçuş** (pxh'ye komut yazmadan): arayüzsüz test + Oğuz'un arayüzlü çalıştırması.
  - Arayüzsüz (hover 5 s): offboard+arm kabulü 1,3 s; tırmanış 8,1 s (0,3 m toleransa); iniş komutundan disarm'a 11,2 s. PX4 logu: "Armed by external command" → "Takeoff detected" → "Landing detected" → "Disarmed by landing".
  - Arayüzlü (hover 10 s): kabul 1,3 s; tırmanış 8,2 s; iniş 11,1 s.

- **Kamera kararı:** 1920×1080, HFOV 60°, 10 Hz (offline tiling'de insan ~25 px; PX4 varsayılanı 115° ile ~6 px olurdu). Kendi Gazebo modellerimiz `sim/gz_models/usm_gimbal` + `usm_x500_gimbal`; overlay betiği bunları da linkliyor; araç dosyası artık `usm_x500_gimbal`'i spawn ediyor.
  - Arayüzsüz doğrulama: model `usm_x500_gimbal_0`, kamera 1920×1080, gz RSS 1,22 GB sabit, VRAM 400 MiB, GPU %18. Kare hızı `gz topic -e` ile ~6 Hz ölçüldü (araç darboğazı olabilir; ROS 2 tarafında `ros2 topic hz` ile tekrar ölçülecek).
- Hata: bir test betiği, açık olan kullanıcı simülasyonunu fark etmeden ölçtü ve kapattı. Test betiklerine "PX4/Gazebo açıksa çalışma" koruması eklendi.

**Sıradaki adım**
- Şerit (lawnmower) rota hesabı (Oğuz, `usm_core`) → görev node'u waypoint'leri uçar → RTL.

**Açık kararlar**
- Kayıt stratejisi (1080p ham ~62 MB/s @10 Hz: JPEG mi, çekim noktalarında fotoğraf mı?)

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
