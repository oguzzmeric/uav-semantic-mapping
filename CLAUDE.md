# uav-semantic-mapping — Proje Rehberi (Claude Code için)

Bu dosya her oturumun başında okunur. Bir oturuma başlarken önce bu dosyayı, sonra `PROGRESS.md`'yi oku ve kaldığımız yerden devam et.

(GitHub repo adı: `uav-semantic-mapping`. Yerel klasör şimdilik `~/uav-sar-geolocation`.)

## 1. Proje özeti

Otonom semantik haritalama drone'u (simülasyon + gerçek veri): drone bir alanı kendi başına tarar, **insan ve araçları** canlı tespit edip yerdeki konumlarını hesaplar, uçuş sonrası alanın 3B haritasını çıkarıp nesneleri bu haritaya katman olarak yerleştirir ve konum doğruluğunu sistematik olarak ölçer.

1. Drone verilen alanı kendi başına tarar: şerit (lawnmower) rotası, nadir kamera (gimbal −90°), fotogrametri için ~%70-80 örtüşme.
2. Gimbal kameradan insan ve araç tespit eder: Oğuz'un TEKNOFEST 2025 için **nadir drone görüntüleriyle eğittiği YOLO11l** modeli (standart `yolo11l.yaml` mimarisi, imgsz 1280, 10 sınıf; projede INSAN ve araç sınıfları kullanılır).
   - **Canlı:** uçuşta TensorRT ile tespit + canlı konum (demo ve önizleme).
   - **Offline:** uçuş sonrası tam karede tespit (tiling yok; 2026-11-05 kararı: `seed 1`'de nesne düzeyinde tiling kazandırmadı ve 3 kat yavaş; irtifa taramasında 40 m'de yeniden denenecek). **Resmi sayılar buradan gelir.**
3. Her tespitin yerdeki koordinatını hesaplar: piksel → ışın → zemin düzlemi kesişimi (EKF2 pozu + ölçülen gimbal açısı + kamera iç parametreleri; sınıfa göre nesne yüksekliği düzeltmesi).
4. Aynı nesnenin farklı karelerdeki tespitlerini tek kayıtta birleştirir.
5. Görev bitince başlangıç noktasına döner ve iner (PX4 RTL).
6. Uçuş sonrası COLMAP ile seyrek 3B harita çıkarır; haritayı Sim(3) ile dünya koordinatına hizalar; nesneleri haritada işaretler ve rapor üretir.
7. Sonuçları ölçer: simülasyonda Gazebo'daki gerçek konumlarla (hata kaynaklarına ayrıştırarak), gerçek veride VisDrone ile.

**Kariyer amacı:** "Otonom sistemler için algılama (perception) mühendisi" kimliğini kanıtlamak. Projenin değeri özellik sayısından değil, **ölçüm derinliği ve dürüstlüğünden** gelir. Proje GitHub'da baştan herkese açık.

## 2. Ortam

- Ubuntu 24.04, ROS 2 Jazzy (`/opt/ros/jazzy`), Gazebo Harmonic (ROS vendor 8.10 + OSRF 8.15; bkz. tuzaklar)
- **PX4 v1.17.0** SITL: `~/PX4-Autopilot`; Python bağımlılıkları `~/px4-venv`'de (sisteme kurulmadı). ROS 2 bağlantısı: Micro XRCE-DDS + `px4_msgs` dalı `release/1.17` (`~/px4_ws`, alt katman)
- **Projenin araç dosyası:** `sim/px4/airframes/4900_gz_usm_x500_gimbal` (4019 x500_gimbal + simülasyon ayarları, ör. `NAV_DLL_ACT=0`). PX4'e link + yamalar: `setup/px4_overlay.sh` (tekrar çalıştırılabilir; yeni kurulumda bir kez çalıştırılır, sonra `make px4_sitl`). Yamalar: `setup/px4_patches/`. Köy dünyasının dokuları: `setup/fetch_textures.sh` (yeni kurulumda bir kez). JPEG kayıt için: `sudo apt install ros-jazzy-compressed-image-transport`.
- **Köy dünyası:** `sim/gz_worlds/make_village.py --seed N` → `village.sdf` + `village_objects.csv` (nesnelerin gerçek konumları: NED, taban merkezi, boy; tüm metriklerin referansı). Elle düzenlenmez, betikten üretilir. Kaynaklar ve lisanslar: `sim/gz_worlds/CREDITS.md`.
- **Çalıştırma (3 terminal, sonra tek launch dosyasına dönüşecek):**
  1. Simülasyon: `~/uav-sar-geolocation/setup/run_sim.sh` (arayüzsüz: `setup/run_sim.sh HEADLESS=1`; köy dünyası: `setup/run_sim.sh PX4_GZ_WORLD=village`). PX4'ün kayıtlı parametrelerini silip `make px4_sitl gz_usm_x500_gimbal`'i NVIDIA'da başlatır; her deney araç dosyasındaki ayarlardan başlar.
  2. Köprü: `~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888` (v2.4.3, kaynaktan derlendi, sisteme kurulmadı)
  3. ROS 2: `source ~/px4_ws/install/setup.bash` (`px4_msgs` `release/1.17`, mesajlar PX4 v1.17.0 ile birebir doğrulandı)
  - Gazebo köprüsü (kamera, gerçek konum, `/clock`; kayıt ve canlı tespit için gerekli, tek bir tane açılır): `ros2 launch usm_mission bridge.launch.py world:=village`
  - Kayıt (repo kökünde, köprü açıkken): `ros2 launch usm_mission record.launch.py` → `data/bags/<tarih>`; dışa aktarma: `python3 tools/export_bag.py <kayıt>`; poz kontrolü: `python3 tools/check_pose.py <kayıt>`.
  - Canlı tespit (repo kökünde, köprü açıkken): `ros2 launch usm_detection live.launch.py [rerun:=false] [run:=<ad>]` → `data/live/<ad>.csv` + `<ad>_gpu.csv`; sayılar: `.venv/bin/python tools/live_stats.py data/live/<ad>.csv`.
  - Görev: `ros2 run usm_mission survey_mission`.
  - Kare düzeyi tespit ölçümü (repo kökünde, `.venv/bin/python`): `tools/truth_boxes.py data/export/<ad> sim/gz_worlds/village_objects.csv` → `tools/detect_offline.py data/export/<ad> --run <ad> [--tiled]` → `tools/eval_detections.py data/export/<ad> --run <ad>`.
  - Nesne düzeyi hat (aynı şekilde): `tools/geolocate.py data/export/<ad> sim/gz_worlds/village_objects.csv --run <ad>` → `tools/merge_objects.py data/export/<ad> --located <ad>_ekf` → `tools/eval_objects.py data/export/<ad> sim/gz_worlds/village_objects.csv --located <ad>_ekf`.
  - Harita (`uv sync --group detect --group map`): `tools/build_map.py data/export/<ad>` → `tools/align_map.py data/export/<ad> sim/gz_worlds/village_objects.csv`.
  - Projenin ROS paketlerini derleme (repo kökünde, `px4_ws` yüklüyken): `.venv/bin/python -m colcon build --base-paths ros --symlink-install`. colcon'u venv Python'uyla çalıştırmak node'ların ilk satırına venv Python'unu yazar; node'lar venv açık olmasa da `usm_core`'u (ve ileride torch'u) görür. Düz `colcon build` sistem Python'unu yazar ve `usm_core` import edilemez.
  - Kapatma: her terminalde **`Ctrl+C`**. Pencereyi X ile kapatmak PX4/Gazebo'yu arka planda açık bırakır.
- GPU: NVIDIA GTX 1650 **4 GB VRAM**, sürücü 595 (`nvidia-driver-595-open`, imzalı çekirdek modülü), CUDA 13.2. Hibrit grafik: PRIME `on-demand` (masaüstünü Intel çizer; Gazebo NVIDIA'da çalıştırılmak için offload ortam değişkenleriyle başlatılır). CPU: i7-11800H (8 çekirdek). RAM: 16 GB. Disk: ~50 GB boş.
- Python: proje içinde `.venv`, **uv** ile yönetilir (kilit dosyası commit'lenir). `rclpy` sistem Python'unda olduğu için venv `--system-site-packages` ile oluşturulur (sistem paketleri okunur, sisteme paket kurulmaz).
- Model ağırlığı: `models/teknofest_yolo11l_nadir.pt` (`.gitignore`'da, repoya girmez)
- Referans (aynı ortamda çalıştığı kanıtlı): Ubuntu 24.04 + Jazzy + PX4 + Gazebo Harmonic → `Hayat373/autonomous-ai-rescue-drone`

**Donanım kısıtları:** 4 GB VRAM nedeniyle canlı tespitte TensorRT fp16; Gazebo + YOLO aynı anda çalışırken VRAM ölçülür. YOLO11l ile ince ayar 4 GB'a sığmayabilir (backbone dondurma, küçük batch, tiling).

**Bilinen tuzaklar:** `cv_bridge` (apt, NumPy 1.x) ile pip'ten gelen NumPy 2.x çakışabilir → NumPy sürümü sabitlenir ve test edilir. TensorRT'nin sürücü 595 / GTX 1650 ile uyumu kurulumda doğrulanır. Çekirdek güncellemesi NVIDIA modülünü bozabilir (2026-10-23'te oldu): `nvidia-smi` çalışmazsa önce çekirdek ↔ `linux-modules-nvidia-*` eşleşmesine bakılır.
- **İki Gazebo:** ROS yüklü kabukta ROS vendor Gazebo'su (8.10), ROS'suz kabukta OSRF Gazebo'su (8.15) devreye girer. PX4 her zaman ROS yüklü normal terminalde derlenir ve çalıştırılır (`ros_gz_bridge` de 8.10'a göre derlenmiş).
- **PX4 `GstCameraSystem` RAM sızıntısı:** eklenti açıkken gz sunucusu ~80 MB/s şişer ve sistem donar (`nvh264enc` sürücü 595 ile başlamıyor + eklentide kuyruk sınırı yok). Eklenti kapalı (`setup/px4_patches/0001-disable-gst-camera-plugin.patch`); kamera görüntüsü `ros_gz_bridge` ile alınır. PX4 yeniden klonlanırsa yama tekrar uygulanmalı.
- **`PX4_PARAM_<ad>=<değer>` ortam değişkeni parametreyi yazılım varsayılanına döndüremez:** PX4'ün `param set`'i değer zaten eşitse hiçbir şey yazmaz; sonra araç dosyasındaki `set-default` kazanır (ör. `NAV_DLL_ACT`: 0 → 2). Simülasyon parametreleri bu yüzden araç dosyamızda `param set-default` ile verilir.
- **Gazebo model adı make hedefinden türetilir** (`gz_<ad>` → `models/<ad>/model.sdf`). Araç dosyasında `PX4_SIM_MODEL` koşulsuz atanmalı (`:=` işe yaramaz, değişken zaten dolu gelir).
- **PX4 topic adlarında sürüm eki olabilir:** ör. `/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_status_v1`. Konum/yönelim **NED** (z aşağı; havadayken z < 0, h = −z).
- **Kamera:** PX4 varsayılanı 1280×720, 30 Hz, HFOV 2,0 rad (~115°; 30 m'de insan ~6 px, kullanılamaz). Projenin kamerası `sim/gz_models/usm_gimbal`: **1920×1080, HFOV 60° (1,0472 rad), 10 Hz** (30 m'de GSD 1,80 cm/px, insan ~25 px). Gazebo modeli `usm_x500_gimbal` (overlay ile PX4'ün model klasörüne linklenir). Görüntüler **JPEG kalite 95** kaydedilir (`/camera/image/compressed`; köyde 0,9 MB/kare, PSNR 38 dB, uçuş başına ~2 GB; ham kayıt 10 GB'tı).
- **PX4 uçuşta öğrendiği parametreleri kalıcı kaydeder** (`build/px4_sitl_default/rootfs/parameters.bson`; ör. pusula sapmaları `CAL_MAG0_*OFF`, `EKF2_MAG_DECL`) ve sonraki açılışta yükler. 2026-10-27'de bozuk öğrenilmiş bir pusula kalibrasyonu yerde 13,6° yön hatasına yol açtı. Bu yüzden simülasyon hep `setup/run_sim.sh` ile açılır (kaydı siler); kalıcı ayarlar sadece araç dosyasında.
- **PX4 zaman damgaları:** `UXRCE_DDS_SYNCT 0` (araç dosyası) ile PX4 mesajları simülasyon saatinde; kamera ve Gazebo odometrisi de sim saatinde. Görev node'u ise duvar saatini kullanıyor (`/rosout` damgaları sim saatiyle uyuşmaz).
- **Büyük mesajlarda `ros2 topic hz` güvenilmez** (Python; 6 MB'lık 1080p karede 5,7 Hz gösterdi, gerçek 10,00 Hz). Kare hızı kayıttaki kare sayısından ölçülür. `timeout` ile durdurulan `ros2` komutları Ubuntu'da sahte "ros2 çöktü" bildirimi açar (zararsız).
- **Fuel modellerinin dokuları Harmonic'te bozuk çizilebilir:** eski OGRE malzeme betikli modeller (ör. House 1-3) **siyah**, dokusu yanlış yolda olanlar (Hatchback) **beyaz**. Çözüm: dünya modeli kendisi tanımlar, dokuyu PBR `albedo_map` ile bağlar (`make_village.py`). Gazebo dokuları **dosya adına göre önbelleğe alır**: aynı adlı iki doku (üç Hatchback'in `hatchback.png`'si) ilk yükleneni gösterir → farklı adlarla indirilir.
- **`image_transport republish`:** `out` yeniden adlandırması `out/compressed`'e geçmez; doğrudan `out/compressed:=...` yazılır. JPEG kalitesi: `out.compressed.jpeg_quality`.
- **`pgrep -f <desen>` kendi kabuğunu da bulur** (desen komut satırında geçiyor) → süreçler `pgrep -x px4` gibi tam adla bulunur, kapatmadan önce `HEADLESS=1` ile benim simülasyonum olduğu doğrulanır.
- **`.venv`'deki sürüm sınırları bilinçli:** `numpy<2` (ROS kütüphaneleri NumPy'nin C arayüzüyle 1.x'e göre derlenmiş), `pyarrow<26` ve `rerun-sdk<0.24` (daha yenileri NumPy 2 istiyor; pyarrow bunu paket bilgisinde yazmadığı için hata ancak `import`'ta çıkar), `setuptools<80` (torch getiriyor; daha yenisinde `colcon --symlink-install`'ın kullandığı `develop --editable` yok).
- **`rr.spawn` viewer'ı PATH'te arar:** venv açık değilken `.venv/bin` PATH'te olmaz, node PATH'e ekler. Rerun yerleşimi diske kaydeder (`~/.local/share/rerun/blueprints/`) ve açılışta geri yükler; yeni yerleşim `send_blueprint(make_active=True)` ile gönderilir.
- **`use_sim_time` + uzun callback:** rclpy `/clock`'u node'un varsayılan callback grubunda işler. Tek iş parçacığında uzun bir callback sırasında sim saati durur ve gecikme ~0 ölçülür; `live_detector` bu yüzden iki iş parçacığı ve ayrı callback grubu kullanır.
- **Ctrl+C `ros2 launch` altındaki sürece iki kez ulaşır** (terminal + launch): kapanış kodu ikinci sinyali yok saymalı.
- **Test betikleri açık bir simülasyona dokunmamalı:** başlamadan önce PX4/Gazebo çalışıyor mu kontrol edilir (2026-10-26'da kontrolsüz bir test Oğuz'un açık simülasyonunu kapattı).

## 3. Kapsam ve öncelik

### 20 Kasım'a kadar (zorunlu, "çekirdek")
- PX4 SITL + Gazebo + `x500_gimbal` + **düz zeminli köy/kasaba dünyası** (insan + araç modelleri, gerçek konumları bilinen)
- Şerit tarama rotası: kamera görüş alanına ve istenen örtüşmeye göre şerit aralığı ve çekim aralığı hesaplanır. Varsayılan h = 30 m (geçici; irtifa taramasıyla kesinleşir)
- Görev sonunda RTL (eve dönüş + iniş)
- rosbag2 (MCAP) kayıt: görüntü + EKF2 pozu + ölçülen gimbal açısı + zaman damgası + Gazebo gerçek pozları; dışa aktarma betiği
- Offline tespit (tam kare; tiling irtifa taramasında denenecek) + VisDrone test-dev ölçümü
- Canlı tespit node'u (TensorRT) + canlı konum; Rerun ile canlı görselleştirme; hız, gecikme ve VRAM ölçümü
- Piksel → zemin koordinatı (Python, gimbal açısı ve sınıf yüksekliği dahil) + pytest
- Tespit birleştirme (konum tabanlı)
- Ölçüm: recall, precision, konum hatası; irtifa taraması (20/30/40 m); tespit ve geometri hatası ayrıştırma; poz/gimbal gürültüsüne hassasiyet analizi
- COLMAP seyrek 3B harita + Sim(3) hizalama + haritada işaretli nesneler + uçuş sonu raporu
- Demo videosu (karma: simülasyon ana bölüm + "real-world" etiketli kısa gerçek veri bölümü), ölçüm tablosu, İngilizce README

### Zaman kalırsa
- Eğik gimbal açısı deneyi (−45°, −60°, −75°): konum hatasının açıya göre değişimi

### 20 Kasım'dan sonra (sırayla)
1. **C++/Eigen konumlandırma modülü** + ROS 2 node; Python sürümüyle doğruluk ve hız karşılaştırması
2. **ByteTrack** ile takip; canlı nesne listesi takip kimlikleriyle
3. Gerçek uçuş (drone erişimi olursa): gerçek dünyada konum hatası ölçümü
4. *(Opsiyonel)* Ortofoto, DSM, geometriden bina katmanı
5. *(Opsiyonel)* Temiz ve belgelenmiş veriyle P2 başlıklı küçük model; mevcut modelle karşılaştırma

### Çıkarıldı
- Arama-kurtarma çerçevesi (HERIDAL, olasılık haritası, canlı "insan bulundu" uyarısı)
- 2D web haritası

## 4. Haftalık plan ve "bitti" ölçütleri

| Hafta | Tarih | İş | Bitti sayılması için |
|---|---|---|---|
| 1 | 22–28 Eki | Kurulum: Gazebo Harmonic, PX4 SITL, Micro XRCE-DDS, ROS 2 köprüsü; `x500_gimbal`; düz köy dünyası; offboard kontrol; şerit rota + RTL; rosbag2 kayıt | Drone kendi kodumuzla kalkıyor, şerit rotayı uçuyor, eve dönüp iniyor. Her kare görüntü + poz + gimbal açısıyla kayıtta, dışa aktarılabiliyor. |
| 2 | 29 Eki–4 Kas | Offline tespit + VisDrone test-dev ölçümü + ezber kontrolü; TensorRT dönüşümü; canlı tespit node'u + Rerun | P/R/AP tablosu (VisDrone + simülasyon). Canlı tespit Hz, gecikme, VRAM sayıları. |
| 3 | 5–11 Kas | Konumlandırma + testler; birleştirme; gerçek konumlarla karşılaştırma betiği; ilk COLMAP haritası + Sim(3) hizalama | Nesne listesi + konum hatası sayıları. Kayıtlı fotoğraflardan hizalanmış 3B harita. |
| 4 | 12–20 Kas | İrtifa taraması; hata ayrıştırma; hassasiyet analizi; haritada işaretleme + rapor; gerçek veri bölümü; demo video; README; yayın | Grafikler + ölçüm tablosu + rapor + demo video + İngilizce README. Repo pinlendi, CV'ye eklendi. |

En büyük takvim riski hafta 1 (PX4/Gazebo/gimbal ilk kez kuruluyor).

**Teslim tarihleri:** hedef **20 Kasım**; yetişmeyen kısımlar için yedek **1 Aralık** (tam video kurgusu, gerçek dünya bölümü, ek analizler).

**Ara kontroller** (tutmazsa sağdaki sadeleştirme uygulanır):

| Tarih | Olmuş olmalı | Tutmazsa |
|---|---|---|
| 4 Kas | Drone kendi kodumuzla şerit uçuyor + canlı tespit çalışıyor | Canlı kısım sadeleşir (TensorRT yerine PyTorch) |
| 11 Kas | Konumlandırma + birleştirme + ilk metrikler | COLMAP minimuma iner (yedek: pozlardan kuşbakışı harita) |
| 14 Kas | **Özellik dondurma** | Bu tarihten sonra yeni özellik yok: sadece ölçüm uçuşları, video, README |

**Kesme sırası:** gerçek dünya bölümü → eğik açı deneyi → hassasiyet analizinin kapsamı → COLMAP kalitesi. Çekirdek (uçuş + tespit + konum + ölçüm) kesilmez.

**Demo videosu yol boyunca çekilir:** her çalışan adımda (ilk şerit uçuşu, ilk canlı tespit, ilk harita) OBS ile kısa kayıt alınır; son haftada montaj ve eksik bölümler kalır.

## 5. Ölçümler ve başarı çıtası

### Simülasyon (gerçek konumlar Gazebo'dan alınır)

**Nesne katmanı** (insan ve araç ayrı ayrı raporlanır):
- **Bulma oranı (recall):** bulunan gerçek nesne / toplam gerçek nesne. **Hedef ≥ %80**
- **Kesinlik (precision):** doğru rapor / toplam rapor
- **Konum hatası:** raporlanan koordinat ile gerçek konum arası yatay mesafe (m), ortalama ve medyan. **Hedef ≤ 2 m**
- Doğru eşleşme kuralı: rapor, gerçek nesneye ≤ 2 m ise doğru (eşik README'de açıkça yazılır)

**Hata analizi:**
- İrtifaya göre (20/30/40 m): recall ve konum hatası
- **Tespit ile geometri hatasının ayrıştırılması:** gerçek konumun kameraya izdüşümüyle elde edilen "ideal piksel" ile hesaplanan konum (sadece geometri hatası) ve model kutusuyla hesaplanan konum (toplam hata)
- Sınıf yüksekliği düzeltmeli ve düzeltmesiz konum hatası
- **Hassasiyet analizi:** gimbal açısı ve poza yapay gürültü (±1°, ±3°, ±5°; rastgele ve sistematik ayrı). Ölçülen hata teorik beklentiyle aynı grafikte: Δd ≈ h·δ/cos²θ (açı hatası), Δd = Δh·tanθ (yükseklik hatası)
- Kestirilen poz (EKF2) ile gerçek poz (Gazebo) farkının konum hatasına katkısı

**Canlı sistem:** tespit hızı (Hz), kareden kutuya gecikme (ms), Gazebo + YOLO birlikte VRAM kullanımı. Hedefler hafta 2'deki ilk ölçümden sonra sabitlenir.

**Görev:** görev süresi (s), çekilen fotoğraf sayısı.

**3B harita:**
- Doğruluk: haritadaki bilinen noktaların (bina köşeleri, işaretler) Gazebo'daki gerçek konumlarına göre hatası (m). Hizalama: COLMAP kamera merkezleri ↔ EKF2 kamera konumları, Sim(3) (Umeyama); COLMAP `model_aligner` ile çapraz kontrol
- Kapsama: haritaya dahil olan fotoğraf oranı, nokta sayısı/yoğunluğu

### Gerçek veri
- **VisDrone test-dev:** precision, recall, AP (pycocotools). Sınıf eşlemesi (model sınıfları → insan/araç) ve IoU eşiği ölçümden önce sabitlenir ve README'de yazılır.
- **Ezber kontrolü:** modelin eğitim verisi belgelenmemiş (TEKNOFEST + bilinmeyen ek kaynaklar; VisDrone olabilir). VisDrone train ile test-dev arasındaki fark ezber **işareti** olarak raporlanır, kanıt olarak değil.
- **Gerçek dünya demo bölümü:** telemetrili açık veri seti (aday: AU-AIR; telemetri, gimbal, kamera parametreleri ve lisans doğrulanacak).

### Kurallar
- Metrik ve değerlendirme yöntemi ayar yapmadan önce sabitlenir. Sonradan değişirse bu kayda geçer.
- Ayar/eğitim verisi ile test verisi ayrılır.
- Sayılar tek bir değerlendirme betiğiyle üretilir. Elle yazılmış sonuç olmaz.
- "İşaret" ile "kanıt" ayrılır, abartılı ifade kullanılmaz. Simülasyon sonuçları "simülasyonda" diye açıkça belirtilir; demo videosunda canlı ve uçuş sonrası ("post-flight") bölümler etiketlenir.

## 6. Çalışma kuralları (Claude Code için)

**Dil:** Açıklamalar Türkçe; kod, yorumlar, commit mesajları ve README İngilizce.

**Kararlar: HER ŞEYİ SOR.** Kod yazmadan, dosya oluşturmadan, paket kurmadan, komut çalıştırmadan önce ne yapacağını ve nedenini kısaca öner, onay bekle. Seçenek varsa 2-3 seçeneği artı/eksileriyle sun. Sorular numaralı düz metin olarak sorulur: ne karar veriliyor, neden önemli, her seçenek pratikte ne demek, öneri.

**Kod yazımı ve sorumluluk (2026-11-05'te değişti).**
- **Oğuz mimariyi şekillendirir:** parçaların yapısı, bağlantıları ve ödünleşimleri ona sorulur. Hesap alıştırması veya kontrol sorusu verilmez.
- **Claude yazar ve matematiği çözer** (mimari kararından sonra): konumlandırma, birleştirme, harita hizalaması, konum metrikleri dahil bütün modüller. Her modülde neden öyle yapıldığı ve sınırları açıklanır; Oğuz bunları mülakatta anlatabilmeli.
- Çekirdek matematik ROS'tan bağımsız saf Python paketi olarak tutulur (CI'da ROS olmadan test edilir; ileride C++'a taşınır).

**Açıklama: DETAYLI.** Her adımda "neden"i anlat. Matematik varsa formülü ve geometrik anlamını açıkla. Kullanılan kavramları (ör. offboard mode, NED/ENU, gimbal açıları, kamera iç/dış parametreleri) ilk geçtiği yerde kısaca tanımla.

**Ölçmeden ilerleme:** Her adım bir ölçüm veya gözle görülür bir doğrulamayla biter. Tek hipotez, tek değişiklik.

**Testler:** Kritik matematik için pytest: piksel → ışın, ışın-zemin kesişimi, koordinat dönüşümleri (NED/ENU, gimbal), sınıf yüksekliği düzeltmesi, tespit birleştirme, Sim(3) hizalama. Bilinen geometrik durumlarla test edilir (ör. tam aşağı bakan kamera, görüntü merkezi → drone'un tam altı; nadirde h hatası merkez noktayı kaydırmaz). CI (GitHub Actions) her push'ta testleri çalıştırır.

**İlerleme kaydı:** Her oturum sonunda `PROGRESS.md` güncellenir: tarih, ne yapıldı, ne ölçüldü (sayılarla), sorunlar, sıradaki adım, açık kararlar.

**Commit'ler:** Küçük ve anlamlı; mesaj İngilizce. Repo herkese açık olduğu için anahtar, şifre, `.env`, büyük veri, kayıtlar ve model dosyaları commit'lenmez (`.gitignore`).

## 7. Teknolojiler

| Katman | Teknoloji |
|---|---|
| Altyapı | Ubuntu 24.04, ROS 2 Jazzy, colcon |
| Simülasyon | Gazebo Harmonic, SDF dünya + Gazebo Fuel modelleri, PX4 `x500_gimbal` |
| Uçuş | PX4 SITL (EKF2, offboard, RTL), Micro XRCE-DDS Agent, `px4_msgs`; QGroundControl (izleme, isteğe bağlı) |
| Köprü ve kayıt | `ros_gz_bridge`, rosbag2 (MCAP), `cv_bridge` + OpenCV |
| Algılama | PyTorch, Ultralytics 8.3.x, YOLO11l, tiling (SAHI veya kendi betiğimiz), ONNX → TensorRT fp16 (yedek: ONNX Runtime GPU → PyTorch fp16) |
| Geometri | NumPy, SciPy (`Rotation`, KD-tree) |
| Harita | pycolmap (CPU, seyrek), Umeyama/Sim(3), COLMAP `model_aligner` (çapraz kontrol) |
| Görselleştirme | Rerun (canlı + post-flight), RViz2 (hata ayıklama), matplotlib (grafikler) |
| Ölçüm ve rapor | pycocotools (AP), kendi konum metrikleri, Jinja2 (HTML/Markdown rapor) |
| Geliştirme | uv + `.venv`, pytest, Git + GitHub Actions |
| Video | OBS Studio (NVENC), ffmpeg / Kdenlive |

## 8. Veri ve lisanslar

- **Repo lisansı: AGPL-3.0** (Ultralytics AGPL-3.0 olduğu için).
- **Ultralytics YOLO:** AGPL-3.0. **PX4:** BSD-3. **ROS 2, Gazebo:** Apache 2.0. **COLMAP:** BSD. **Rerun:** MIT/Apache 2.0.
- **Gazebo Fuel modelleri:** lisans modele göre değişir (çoğu CC-BY); README'de kaynak gösterilir.
- **Model ağırlıkları:** TEKNOFEST verisiyle eğitildi, yarışma verisinin koşulları belli değil → **repoya konmaz.** Sonuçlar, metrikler ve video paylaşılır. README'de "kendi ağırlığını ver veya açık bir YOLO11 modeli kullan" seçeneği yazılır. TEKNOFEST görüntüleri README'de gösterilmez.
- **VisDrone:** akademik/ticari olmayan kullanım. Veri repoya girmez, README'de indirme adımları yazılır; README'de VisDrone görüntüsü gösterilmez.
- **Gerçek dünya demo veri seti:** lisansı kullanmadan önce kontrol edilecek.
- Veri setleri ve uçuş kayıtları repoya girmez.

## 9. Riskler ve yedek planlar

| Risk | Yedek plan |
|---|---|
| Hafta 1 kurulumu uzar (PX4/Gazebo/gimbal ilk kez) | Referans repo (`Hayat373/autonomous-ai-rescue-drone`) kurulum adımları; gimbal sorun çıkarırsa önce sabit nadir kamerayla ilerle |
| Köy dünyası + gimbal GTX 1650'de yavaş | Dünyayı sadeleştir (daha az bina/ağaç, düşük kamera çözünürlüğü) |
| `px4_msgs` ile PX4 sürümü uyuşmazlığı | İkisi aynı sürüm etiketine kilitlenir |
| NumPy 2.x ile `cv_bridge` çakışması | NumPy sürümü sabitlenir; gerekirse `cv_bridge` yerine elle dönüşüm |
| TensorRT kurulmaz / GPU ile uyumsuz | ONNX Runtime GPU, sonra PyTorch fp16 |
| Canlı tespit + Gazebo 4 GB VRAM'e sığmaz | Kamera çözünürlüğü, Gazebo grafik kalitesi, canlıda tiling kapalı; ölçüm zaten offline |
| Model simülasyonda zayıf (gerçek ↔ sim farkı) | Kısa ince ayar (4 GB sınırları dahilinde); iki ortamın sonuçları ayrı raporlanır |
| Küçük insanlar (h = 30 m'de ağ girişinde ~17 px) kaçırılır | Offline'da tiling; gerekirse irtifa düşürülür (irtifa taraması karar verir) |
| COLMAP Gazebo görüntülerinde eşleşme bulamaz (az doku) | Dünyaya dokulu zemin/bina malzemeleri ekle; örtüşmeyi artır; bilinen pozları başlangıç olarak ver; gerekirse SuperPoint+LightGlue (hloc) |
| Telemetrili gerçek veri seti bulunamaz / lisans uygun değil | Gerçek bölüm sadece tespit gösterir (konum yok), bu açıkça yazılır |
| Zaman yetmez | Önce eğik açı deneyi çıkar, sonra hassasiyet analizinin kapsamı daralır; çekirdek korunur |

## 10. Referanslar ve fark noktaları

- SAGE-UAV (PX4 + ROS 2 + Gazebo + YOLO; ışın atarak konumlandırma, %89–100 bulma, 0,30–0,46 m hata): https://github.com/haidar996/SAGE-UAV
- autonomous-ai-rescue-drone (Ubuntu 24.04 + Jazzy + PX4 + Gazebo Harmonic + YOLO11 + ByteTrack): https://github.com/Hayat373/autonomous-ai-rescue-drone
- sar_drone (PX4 + MAVSDK, batarya hesaplı kapsama rotası): https://github.com/rudreshjani29/sar_drone
- PX4 ROS 2 kullanıcı rehberi: https://docs.px4.io/main/en/ros2/user_guide.html
- COLMAP: https://colmap.github.io/
- Rerun: https://rerun.io/
- VisDrone: https://github.com/VisDrone/VisDrone-Dataset

**Fark noktamız** (bu projeyi benzerlerinden ayıran):
1. Konum hatası sistematik ölçülüyor ve kaynaklarına ayrıştırılıyor (irtifa, gimbal açısı, poz kestirimi, tespit, nesne yüksekliği); ölçülen hata teorik beklentiyle karşılaştırılıyor.
2. Canlı (TensorRT) ve offline tespit ayrı ayrı ölçülüyor; gecikme ve VRAM sayılarıyla.
3. Nesneler, drone'un kendi fotoğraflarından üretilen ve dünya koordinatına hizalanan 3B haritada işaretleniyor.
4. Model simülasyonda ve gerçek veride (VisDrone) ayrı ölçülüyor; eğitim verisi belirsizliği açıkça raporlanıyor.

## 11. Geliştirici hakkında (bağlam)

- Yazılım mühendisliği yeni mezunu. Bitirme projesi: GPS'siz drone için monoküler görsel odometri (SuperPoint+LightGlue, Sim(3) ATE/RPE ile ölçüm): https://github.com/oguzzmeric/monocular-vo-scale-study
- TEKNOFEST 2025 Havacılıkta Yapay Zeka finalisti; nesne tespitinde en yüksek ikinci skor.
- Python ve OpenCV güçlü; C++ temel; ROS 2 tanıdık (kurs düzeyi); PX4/Gazebo/MAVLink ilk kez kullanılacak, bu yüzden adım adım ve kavramları açıklayarak ilerle.
- Günde ~3 saat (hafta içi). 20 Kasım'a kadar ~60 saat.
- Drone erişimi yok (2026-10-23 itibarıyla). Yakın bir iş başvurusu tarihi yok.
