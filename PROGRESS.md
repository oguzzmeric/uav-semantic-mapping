# PROGRESS

Her oturum sonunda en üste yeni bir kayıt eklenir.

---

## 2026-11-04 — Canlı tespit uçuşta çalıştı (4 Kasım ara kontrolü geçti)

**Yapıldı**
- Adım 2 yeniden anlatıldı. Konu 4a ve Konu 5'in üstüne canlı tespit tasarımı uygulandı.
- `bridge.launch.py`: köprü 3 tek başına. `record.launch.py`: JPEG + kayıt, kamera yayında değilse uyarı, `/detections` kayda ekli.
- `ros/usm_detection`: `live_detector.py` (en yeni kare, `/detections`, Rerun, kare başına CSV) + `live.launch.py` (`use_sim_time`, `run:=<ad>`, GPU kaydı).
- `tools/sample_gpu.py` (süreç başına VRAM, yük, sıcaklık, saat, kısıtlama) ve `tools/live_stats.py` (sabitlenen yöntemle tarama penceresi sayıları).
- `setuptools<80`: torch setuptools 84 getirmişti, `colcon build --symlink-install` 28 Ekim'den beri bozuktu (o günden beri derleme yapılmadığı için fark edilmedi).
- CLAUDE.md: çalıştırma komutları ve yeni tuzaklar.
- Kare düzeyi ölçüm: `usm_core/projection.py` + `mesh.py` + `tools/truth_boxes.py` (mesh'ler gerçek pozla izdüşürülür → `truth_coco.json`), `tools/detect_offline.py` (`--tiled`), `tools/eval_detections.py` (pycocotools). Değerlendirici önce cevabı elle bilinen küçük veride sınandı. Toplam 52 test geçiyor.
- Konu 5b dersi verildi; `pycocotools` 2.0.11 `detect` grubuna eklendi.

**Ölçüldü** (simülasyonda, PyTorch fp32, imgsz 1280, arayüzlü Gazebo; tarama penceresi: offboard + 30 m)

| Uçuş | Kare | Hz | Atlanan | Toplam gecikme medyan / p95 (ms) | Ağ medyan (ms) | Rerun (ms) | Tarama sıcaklığı | RTF |
|---|---|---|---|---|---|---|---|---|
| `no_viewer` | 399 | 2,29 | %77 | 508 / 812 | 285 | yok | ort. 84,7°C | 0,987 |
| `with_viewer_2` | 402 | 2,30 | %77 | 428 / 704 | 232 | 68 | ort. 81,6°C | 0,990 |
| İlk uçuş (eski kod, tüm iz her karede) | 325 | 1,87 | %81 | 512 / 831 | 286 | 81 | ölçülmedi | ölçülmedi |

- VRAM en fazla: `no_viewer` 2237 MiB, `with_viewer_2` 2518 MiB (Gazebo sunucusu 797, arayüz ~711, node 700, Rerun 278).
- Görev tamamlandı: tarama ~174 s; RTL + iniş; kutu/kare: insan ~1,37, araç ~1,10.
- **Uçuş boyunca yavaşlama (açıklanamadı):** her iki uçuşta da ağ süresi tarama boyunca artıyor (`no_viewer` 220 → 336 ms, `with_viewer_2` 200 → 293 ms; Hz ~2,7 → ~2,0). GPU saati düşmüyor (~1700 MHz). Güç sınırı (`sw_power_cap`) örneklerin ~%70'inde, ama payı tarama boyunca artmıyor. Isı kısıtlaması `no_viewer`'da az, `with_viewer_2`'de taramada yok.
- İki uçuş Rerun'un Hz'ye etkisini ayıramadı: Rerun'lu uçuş daha serin başladı, ağ süresi daha düşük çıktı. Tek karşılaştırma; kanıt değil.
- Kısa testte (yerde, Agent kapalı) `rerun:=false` 3,18 Hz, gecikme medyanı 374 ms: uçuştakinden iyi.

- **Yavaşlama deneyi (ROS ve simülasyon yok, aynı 120 kare, 5 dk, 2537 kare):** ağ 109 → 113 ms (%4), 8,67 → 8,31 Hz; GPU 65 → 81°C, CPU paketi 95°C, GPU hep `sw_power_cap`. Uçuştaki ~%50'lik yavaşlama YOLO'nun kendisinden gelmiyor.
- **Kare düzeyi tespit, `village_seed1_alt30` (geliştirme verisi, simülasyonda, 1737 kare, IoU ≥ 0,5, P/R skor ≥ 0,25):**

| Çalışma | Sınıf | Gerçek | TP | FP | FN | P | R | AP50 | AP50-95 |
|---|---|---|---|---|---|---|---|---|---|
| Tiling'siz | insan | 2836 | 2307 | 89 | 529 | 0,963 | 0,813 | 0,948 | 0,605 |
| Tiling'siz | araç | 1576 | 1572 | 4 | 4 | 0,997 | 0,997 | 1,000 | 0,772 |
| Tiling'li | insan | 2836 | 2492 | 235 | 344 | 0,914 | 0,879 | 0,941 | 0,610 |
| Tiling'li | araç | 1576 | 1561 | 4 | 15 | 0,997 | 0,990 | 0,990 | 0,797 |

- Nesne başına karede bulunma: her nesne en az 61 karede bulundu. `person_01` tiling'siz 61/113 (%54) → tiling'li 91/113 (%81). Aynı 3B modelde bile oran çok değişiyor (Standing person tiling'siz %54–%99); sebebi ölçülmedi.
- Hız, Gazebo kapalı: tiling'siz medyan 116 ms/kare.

**Sorunlar**
- Rerun açılışta kendi kaydettiği yerleşimi yükledi, harita paneli görünmedi → `send_blueprint(make_active=True)`. Düzeltme gerçek pencerede henüz görülmedi.
- Ctrl+C iki kez ulaştığı için kapanışta `Traceback` → ikinci sinyal yok sayılıyor; 3 denemede temiz kapandı. Kapanışta `exception was never retrieved` mesajı kalıyor (süreçler temiz bitiyor, sebebi incelenmedi).
- Uçuş 1 öncesi "Rerun en güçlü şüpheli" demiştim; `no_viewer` uçuşu bunu zayıflattı.

**Sıradaki adım**
- Yeni Rerun yerleşimi ve renkleri açılışta pencerede kontrol edilecek (yerleşim bu oturumda `⋯` menüsüyle gelmişti).
- VisDrone test-dev + ezber kontrolü (diskte 18 GB boş). TensorRT için öneri: atla, resmi sayıları etkilemiyor; karar Oğuz'da.
- Konu 2b + 6a + 3'ün gimbal kısmı → ⑥ konumlandırma (11 Kasım ara kontrolü).

**Kararlar**
- Canlı ekrana harita eklenmedi: panel iz olarak kalır, ⑥'dan sonra nesne noktaları eklenir. Asıl harita COLMAP ile uçuş sonrası.
- Kare düzeyi ölçüm kuralları, sonuçlardan önce sabitlendi: IoU ≥ 0,5 (AP50 + AP50-95, pycocotools); P/R skor ≥ 0,25; kareler offboard + 30 m + nadirden ≤ 10°; kutusunun %50'sinden azı karede olan ya da %50'den çoğu ev altında kalan nesne sayılmaz; evin üstündeki kutu yanlış alarm.
- **Kural değişikliği:** gerçek kutular başta 3B prizmadan türetildi; insanlarda fazla bol çıktı (66 × 55 px; doğru bir kutunun IoU'su ~0,31) → mesh dış zarfına geçildi (48 × 37 px). Karar P/R görülmeden verildi, ama önceki gösterimlerden birkaç model kutusu biliniyordu.
- `seed 1` artık geliştirme verisi (tiling buna bakılarak seçildi). **Resmi sayılar yeni köyden (`--seed 2`) ve yeni uçuştan gelecek**; ayarlar ondan önce sabitlenir. `make_village.py` `village_objects.csv`'nin üzerine yazar, köy dosyaları ayrı saklanmalı.
- Tiling: 2 parça 1280 × 1080, 640 px örtüşme, iç kenara değen kutu atılır, sınıf başına NMS 0,7 (sonuç görülmeden sabitlendi).

**Açık kararlar**
- Tiling'li mi tiling'siz mi? AP neredeyse aynı; nesne düzeyi ölçüm (⑥ + ⑦) belirleyecek. `seed 2`'den önce kapanmalı.
- Canlı konum: ayrı node mu, `live_detector` içinde mi? (⑥ bitince)
- Canlıdaki yavaşlamanın kaynağı (ROS mu, simülasyon yükü mü) ayrılmadı; yalnızca önizlemeyi etkiliyor, öncelik düşük.

---

## 2026-11-03 — Mimari turu, tespit çekirdeği, Rerun kurulumu (Hafta 2)

(28 Eki akşamı ve 30 Eki dahil.)

**Yapıldı**
- (28 Eki akşam) Tespitli uçuş videosu `data/videos/village_seed1_alt30_detections.mp4`: kayıtlı uçuş, YOLO fp32 1280, kuşbakışı harita paneli (scratchpad betiği, repoda değil). Oğuz'un aynı akşamki uçuşu kaydedilmedi; kayıt launch'ı başlatılmamıştı. Ders: komutlar tek seferde verilmeli → `survey.launch.py` fikri.
- (30 Eki, 3 Kas) **Mimari turu**, canlı gösterimlerle: Konu 1 (büyük resim: Gazebo/PX4/Agent/ROS 2/kaptan, 3 köprü), 2a (EKF2 ve Kalman sezgisi), 5 (YOLO, GPU, fp16/fp32, canlı ve offline), 4a (zaman damgaları, sim/duvar saati, QoS). Kalan konular kullanılacakları işten hemen önce anlatılacak:
  - 5b (P/R/AP): offline ölçümden önce;
  - 2b + 6a + 3'ün gimbal kısmı: ⑥ konumlandırmadan önce;
  - 3'ün kalanı: `survey.launch.py`'den önce;
  - 6b: COLMAP'ten önce.
- Canlı tespit tasarım kararları (aşağıda, "Kararlar").
- `usm_core/detection.py`, tespit çekirdeği (canlı node ve offline betik ortak kullanacak):
  - `Detector`: YOLO, fp32, imgsz 1280, conf 0,25, NMS IoU 0,7; `warmup()`; aşama süreleri.
  - `Settings` (dondurulmuş), `Detection` (sade kutu), `CLASS_MAP`.
  - Ultralytics `Detector` oluşturulurken import ediliyor; böylece CI torch'suz test edebiliyor.
  - `tests/test_detection.py`: 4 test (eşleme, atılan sınıflar, yazım hatası koruması, import torch yüklemiyor). Toplam **26 test geçiyor**.
- `rerun-sdk` 0.23.1 `detect` grubunda; `pyarrow<26` (bkz. Sorunlar).
- Oğuz `coverage.py`'deki kullanılmayan `from turtle import width` satırını sildi (tkinter'sız makinede import'u bozardı).

**Ölçüldü**
- EKF2, yerde, canlı:
  - `eph` 0,15 m, `epv` 0,17 m, `evh` 0,09 m/s;
  - `heading_var` 0,034 rad² (≈ ±10°), `heading_good_for_control: false`: yerde yön sadece pusuladan;
  - dışarı verilen hızlar: `sensor_combined` 99,4 Hz, `vehicle_local_position` 49,7 Hz.
- RTF, 3 saat açık kalan simülasyon: PX4 sim saati 11 135 s, duvar saati 11 207 s → **0,994**.
- PX4 `/fmu/out` QoS: BEST_EFFORT, TRANSIENT_LOCAL. Yayıncı `_CREATED_BY_BARE_DDS_APP_` (Agent; bir ROS node'u değil).
- VRAM, arayüzlü köy simülasyonu, YOLO yokken:
  - toplam 1359 / 4096 MiB: gz sunucusu 771, **gz arayüzü 564 MiB** (`HEADLESS=1` ~0,5 GB kazandırır);
  - GPU %38.
- YOLO anatomisi, kare 1600 (`village_seed1_alt30`):
  - 1920×1080 → 1280×736;
  - ham çıktı (1, 14, 19 320) = 160×92 + 80×46 + 40×23 aday;
  - 0,25 üstü 77 aday → NMS sonrası 8 kutu (6 araç, 2 insan; insanlar ağ girişinde 25×17 ve 31×23 px);
  - alt kenarda yarısı kare dışında kalan bir insan kaçırıldı.
- Hız, Gazebo aynı GPU'da açıkken:
  - ham ağ 165–169 ms;
  - `Detector`: ön işleme 7–9 / ağ 150–157 / son işleme 2–19 ms → **~6 kare/s, kamera 10 Hz** → canlıda kare atlanacak (depth 1). Sürenin ~%90'ı ağın kendisi.
  - 28 Ekim'deki 113–119 ms büyük ihtimalle Gazebo kapalıyken ölçülmüştü.
  - CPU'da ağ 1466 ms.
  - YOLO tensörleri 403 MB (PyTorch'un ayırdığı; süreç toplamı değil).

**Sorunlar**
- Rerun kurulumu pyarrow 26'yı getirdi; pyarrow 26 NumPy ≥ 2 istiyor (`import rerun` hatası). NumPy 1.x'te kalıyoruz (`cv_bridge`, ROS mesaj kütüphaneleri). pyarrow 21–25 ayrı bir geçici ortamda NumPy 1.26 ile denendi, hepsi çalıştı → `pyarrow<26`. uv çözücüsüne göre rerun-sdk ≥ 0.23.2 NumPy ≥ 2 istiyor → `rerun-sdk<0.24`.

**Sıradaki adım (onaylandı, buradan devam)**
- **Önce Adım 2'yi yeniden anlat** (Oğuz'un isteği): Rerun nedir; paket, bağımlılık ve sürüm sabitleme; NumPy 1.x/2 uyumsuzluğu neden ROS'u kırar; pyarrow hatası nasıl teşhis edilip çözüldü.
- **Adım 3:**
  - `ros/usm_mission/launch/bridge.launch.py`: köprü 3 tek başına (kamera, `camera_info`, odometri, `/clock`).
  - `record.launch.py`'den köprü çıkarılır; JPEG + kayıt kalır.
  - Kayda `/detections` eklenir.
  - Kamera yayında değilse kaydedici uyarı basar.
  - Doğrulama: `ros2 topic hz /clock`, `/camera/image` listede.
- Adım 4: `ros/usm_detection` paketi (`live_detector.py` + `live.launch.py`).
- Adım 5: `tools/live_stats.py` (CSV → Hz, gecikme, atlanan kare).
- Adım 6: CLAUDE.md çalıştırma komutları, commit.
- Sonra: Konu 5b → offline tespit + simülasyonda P/R; VisDrone; TensorRT (gerekirse ilk kesilecek iş).
- 4 Kasım ara kontrolü: şerit uçuşu ✅, canlı tespit henüz yok (hedef: PyTorch fp32).

**Kararlar (sabitlendi)**
- Canlı tespit:
  - (1a) çekirdek `usm_core/detection.py`, node yeni `ros/usm_detection` paketinde;
  - (2a) köprü 3 ayrı launch dosyasında, JPEG kayıtla kalır;
  - (3a) `/detections`, `vision_msgs/Detection2DArray`, damga = **karenin** damgası;
  - (4) Rerun: 960×540 JPEG görüntü, kutular, EKF2 izi, Hz/gecikme grafikleri; zaman çizelgesi sim saati; viewer bellek sınırı 2 GB.
- Canlı ölçüm yöntemi (ölçümden önce sabitlendi):
  - pencere: offboard + 30 m (`check_pose` ile aynı);
  - Hz = işlenen kare / süre, yanında atlanan kare oranı;
  - gecikme: toplam ve taşıma sim saatinde (`use_sim_time`); ön işleme/ağ/son işleme duvar saatinde; medyan / p95 / maks + RTF;
  - VRAM: `nvidia-smi` 1 Hz, süreç bazında, en yüksek değer;
  - kare başına CSV `data/live/<tarih>.csv`;
  - ayarlar offline ile aynı (`Settings`).

**Açık kararlar**
- Canlı konum: ayrı bir node (`live_geolocator`, `/detections` + EKF2) mı, `live_detector`'ın içinde mi? 3. hafta, ⑥ bitince.
- Simülasyonda tespit ölçümünün gerçek kutuları ve IoU kuralı (Konu 5b'de).

---

## 2026-10-28 — Değerlendirme araçları, köy dünyası, JPEG kayıt, ilk tespit (Hafta 1 bitti)

**Yapıldı**
- `tools/check_pose.py <kayıt>`: EKF2 ↔ Gazebo gerçek pozu (eksen başına sistematik/saçılma, yatay medyan/p95/maks), zaman kayması taraması (−300…+300 ms), yaw hatası (tarama + sadece şerit). Ortak okuma/dönüşüm kodu `tools/bagio.py` (commit `3ac6723`).
- `tools/export_bag.py <kayıt>` → `data/export/<ad>/`: `images/` (her kare), `frames.csv` (kare başına sim saati, uçuş modu, EKF2 konumu + yönelim kuaterniyonu, gimbal kuaterniyonu, gerçek poz; hepsi kare anına enterpolasyonlu, NED/FRD, (w,x,y,z)), `camera.json` (iç parametreler) (commit `30c2c72`). Kararlar: yönelim kuaterniyon (nadirde Euler gimbal kilidine düşer), OpenCV.
- Köy dünyası: `sim/gz_worlds/make_village.py --seed N` → `village.sdf` + `village_objects.csv` (gerçek konumlar: NED, **taban merkezi**, boy/uzunluk/genişlik). Seed 1: 20 insan (4 model), 12 araç (yollarda, yol yönünde; 5 model), 3 ev (kenarlarda), artı şeklinde iki 7 m'lik yol, 20 m'lik dokulu zemin karoları (ambientCG CC0, %70 Ground037 + %30 Grass004, rastgele döndürülmüş). İnsan/araç merkezleri arası ≥ 4 m (2 m eşleşme eşiğinin iki katı). `setup/fetch_textures.sh` dokuları indirir (gitignore'da), `px4_overlay.sh` dünyaları linkler, `sim/gz_worlds/CREDITS.md` lisanslar (tek CC-BY: MaleVisitorOnPhone) (commit `a16cf3b`, `3b0d664`).
- Kayıt JPEG kalite 95 (`image_transport republish` → `/camera/image/compressed`); dışa aktarma JPEG baytlarını değiştirmeden yazar (ham kayıtlar için PNG yolu duruyor) (commit `9a4fc66`). `ros-jazzy-compressed-image-transport` kuruldu.
- Tespit paketleri `uv` `detect` grubunda (`uv sync --group dev --group detect`): torch 2.10.0+cu128, Ultralytics 8.3.253 (model 8.3.155 ile eğitildi), NumPy < 2. CI torch'suz kalır (commit `d3399c1`). `.venv` 7,4 GB.
- CLAUDE.md: köy dünyası, kayıt/dışa aktarma komutları, yeni tuzaklar (commit `b87a8d5`).

**Ölçüldü**
- `check_pose` (boş dünya): yatay medyan 0,069 m, p95 0,123, maks 0,180 m; sistematik K/D/A +0,010/+0,017/−0,033 m; en iyi kayma 0 ms; şeritte yaw maks 1,42° (std 0,48°). Elle yapılan önceki analizle tutarlı.
- Dışa aktarma doğrulaması (1751 tarama karesi): CSV'den EKF − gerçek yatay medyan 0,069 m ve yaw `check_pose` ile birebir; kameranın nadirden sapması ort. 1,00° (görev node'u 0,99°); kamera "sağ" ekseni ↔ drone "sağ" ekseni ort. −0,15° → **gimbal kuaterniyonu dünyaya göre ve gerçek pozla aynı kuralda** (PX4 `GZGimbal`: kamera IMU'sunun Gazebo yönelimi, büyük ihtimalle gürültüsüz → simülasyonda gimbal hatası ≈ 0; hassasiyet analizinde yapay gürültü şart).
- Tarama penceresinin (offboard + 30 m) ilk ~14 karesi gimbal henüz dönerken çekiliyor (~90° sapma) → konumlandırma kareleri nadir sapmasıyla da süzmeli.
- Model orijinleri: hepsi zeminde. Taban merkezi ↔ orijin: insanlarda ≤ 0,08 m, Hatchback'te 0,34 m → betik nesneleri taban merkezine göre yerleştirir. Boylar: insan 1,67–1,92 m, araç 1,57–2,16 m (`village_objects.csv`; sınıf yüksekliği düzeltmesi için).
- Köy dünyası: açılış 16–43 s (ilk açılışta Fuel indirmesi), **RTF 0,99**, Gazebo **753 MB VRAM**, GPU %19–29. Uçuş/poz sayıları boş dünyayla aynı (EKF yatay medyan 0,070 m, şeritte yaw maks 1,47°, gimbal şeritte 0,64°).
- Kayıt boyutu (köy, ~2450 kare): ham 6,2 MB/kare → bag 10 GB (zstd ~4 MB/kare); PNG 3,3 MB (8,1 GB); **JPEG q95 0,89 MB, PSNR 38,3 dB → bag 2,1 GB**; q90 0,61 MB / 36,2 dB. Kamera iki biçimde de 10,000 Hz, kare kaybı yok. Dışa aktarma (JPEG) 8 s.
- **İlk tespit** (Oğuz'un YOLO11l'i, köy kareleri, gözle): imgsz 1280 **fp32** 113–119 ms/kare; iki karede görünen bütün araçlar (0,89–0,93; kesik bir araba 0,42) ve 6 insan (0,60–0,78; kutu 31–65 px) bulundu, ev/yol/çimde yanlış tespit yok. `.venv` (8.3.253) ile `~/.local` (8.4.23) aynı kutuları verdi. Resmi ölçüm değil (2 kare).
- **fp16 GTX 1650'de kullanılamaz:** imgsz 1280'de çıktı NaN (hiç tespit yok), 640'ta çalışıyor ama fp32'den 4× yavaş (154 vs 36 ms; 1280'de 513 vs 113 ms). Kart (TU117) tensör çekirdeksiz. Not: Ultralytics ilk `predict`'teki `half` ayarını sonraki çağrılarda yeniden kullanır → karşılaştırmada model her ayar için yeniden yüklenmeli.

**Sorunlar**
- Fuel modelleri Harmonic'te: House 1-3 siyah (OGRE malzeme betiği), Hatchback'ler beyaz (doku yolu bozuk) → dünya bu modelleri kendisi tanımlıyor, doku PBR `albedo_map`. Üç Hatchback'in dokusu aynı adlı (`hatchback.png`) ve Gazebo dokuyu ada göre önbelleğe aldığı için hepsi beyaz çıktı → farklı adlarla indiriliyor. Male visitor bir aktör (animasyonlu, 1 m kaydırılmış) → MaleVisitorOnPhone; Prius eklemli sürülebilir araç → çıkarıldı.
- Araç yerleştirme ilk sürümde takıldı (boşluk iki taraftan ekleniyordu, yollar doldu) → boşluk sadece adaya; yol 7 m (iki 3,5 m şerit), araçlar arası 0,5 m. 20/20 seed yerleşiyor.
- `republish`: `out` yeniden adlandırması `out/compressed`'e geçmiyor → doğrudan `out/compressed:=...`.
- `timeout` ile kestiğim `ros2 topic hz` Ubuntu'da "ros2 çöktü" bildirimi açtı (zararsız).
- `data/bags/village_seed1_alt30` (3,3 GB, 6376 kare): `uv` kurulumu turumu bloke ettiği için kayıt inişten ~6 dk sonra durduruldu; tarama kareleri tam, fazlası yerde.
- Torch/Ultralytics/OpenCV ayrıca `~/.local`'de de kurulu (pip --user); `.venv` önce geldiği için devrede değil.

**Sıradaki adım (Hafta 2)**
- Simülasyonda tespit ölçüm yöntemi kararı: kare başına gerçek kutular nereden? (Gazebo bounding-box kamera sensörü / gerçek konumların izdüşümü) + eşleşme kuralı (IoU) ölçümden önce sabitlenir.
- Offline tespit betiği (tam çözünürlük + tiling, fp32) ve köy kayıtlarında P/R; VisDrone test-dev (pycocotools) + ezber kontrolü.
- TensorRT kurulumu ve fp16/fp32 ölçümü (fp16 PyTorch'ta bozuk; TensorRT'de ayrıca bakılacak); canlı tespit node'u + Rerun.

**Açık kararlar**
- Sınıf eşlemesi **sabitlendi:** insan ← INSAN; araç ← ARABA, KAMYON, OTOBUS, MOTOR; UAP, UAI, MAKINE, GEMI, TREN yok sayılır.
- Harita (Hafta 3'te şekillenecek, ön tercihler): pycolmap (CPU), SIFT ile başla (başarısızsa SuperPoint/ALIKED + LightGlue, lisans kontrolüyle), konuma göre eşleştirme, EKF pozları COLMAP'e **verilmez** (Umeyama ile bağımsız hizalama), sadece seyrek harita. Loop closure ayrıca gerekmez (konuma göre eşleştirme + global BA).
- Canlı tespitin hassasiyeti (fp16 bu kartta bozuk; TensorRT fp16/fp32 ölçümü sonrası).
- Ultralytics `~/.local` kopyalarının kaldırılması (Oğuz'un ortamı, istenirse).

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

- **Yaw incelemesi** (aynı kayıt, `vehicle_attitude` quaternion'undan yaw ↔ gerçek, offboard'dan itibaren 10 s'lik dilimler): 0–10 s +15,7° (maks 38,9°; drone 90° → 0° dönüyor), 10–20 s +12,2° (maks 16,2°), 20–30 s −4,0° (maks 8,1°), 30–100 s ~+1° (maks ≤ 2,1°), 100 s sonrası ~0,2° (maks ≤ 0,8°). Yani 2,2°'lik saçılma **uçuş başındaki gerçek EKF2 yön hatası**: yerde pusulayla ~12° yanlış başlıyor, hareketle (GPS hızı) 30–100 s'de oturuyor; drone bu sürede gerçekten ~10° yanlış yöne bakıyor. Tarama offboard'dan ~17 s sonra başladığı için **ilk şerit bu hatayla uçuluyor** (12° → görüntü kenarında ~3,5 m konum hatası).

- **Yön hatası kök nedeni (devam oturumu):** PX4 uçuşta öğrendiği pusula sapmalarını kalıcı kaydetmiş (`CAL_MAG0_X/Y/ZOFF` = 0,0345 / 0,0296 / 0,1064 G, `parameters.bson`). Kanıt: kalibre ölçüm = ham − bu sapmalar (birebir); kalibre alan 0,408 G / eğim 52,2° (referans 0,482 G / 63,2°). Temiz kayıtla: sapmalar 0, alan 0,481 G / eğim 63,3°; yerde yön hatası 13,6° → ~6°.
  - Çözüm: `setup/run_sim.sh` her açılışta PX4'ün kayıtlı parametrelerini siler (tekrarlanabilirlik de sağlar). CLAUDE.md Terminal 1 komutu buna çevrildi.
  - Yanlış yollar (kayda geçsin): (1) Gazebo pusulasının sapmasını −2,63° hesapladım ve `EKF2_MAG_DECL` ile verdim → yerde 96,1° (daha kötü); işareti yanlış yorumlamışım, alan PX4'ün dünya modeliyle (+3,4°) ~0,6° içinde uyumlu. Geri alındı. (2) `EKF2_DECL_TYPE 0` "her zaman `EKF2_MAG_DECL` kullan" demiyor: kod (`Ekf::getMagDeclination`) bu değeri sadece bit 1 açıkken okuyor; 0'da sapma hiç uygulanmıyor (yerde 92,8°). Parametre açıklaması ile kod uyuşmuyor (upstream'e bildirilebilir).
  - Yerde kalan ~6°: üç ayarda 92,8 / 96,1 / 95,9° — basit bir modelle açıklanamadı (muhtemelen GPS'ten önceki ilk hizalama); düşük öncelik.
- **Havadaki etki (asıl ölçüt), temiz başlangıç, aynı görev:** offboard'dan 10–20 s: +9,5° (tırmanış + ilk şeride ~48 m geçiş); 20–30 s: −0,1° / maks 0,5° (önce −4,0° / 8,1°); sonrası ≤ 1,3°. **Sadece şeritte** (30 m, |vx| ≥ 2, |vy| < 0,5 m/s; 10 723 örnek): ort +0,04°, std 0,39°, p95 0,84°, **maks 1,25°** (bozuk kalibrasyonla: +0,46° / 0,57° / 1,47° / 1,97°). Konum: yatay medyan 0,067 m, maks 0,18 m.
  - **Düzeltme:** yukarıdaki "ilk şerit bu hatayla uçuluyor (~3,5 m)" yorumu yanlıştı — büyük hata ilk şeride giden geçişte; geçişteki yatay hareket yönü oturtuyor, şeritlerde hata bozuk kalibrasyonla bile ≤ 2°'ydi. 10 s'lik dilimler şerit ile geçişi ayırmadığı için yanılmışım.
  - "Yön oturdu" kapısı şimdilik gereksiz. Risk: kalkış noktası ilk şeride çok yakınsa geçiş kısalır → o zaman kapı (ısınma manevrası / EKF yaw belirsizliği eşiği).

**Sıradaki adım**
- Karşılaştırma betiklerini (EKF ↔ gerçek konum, şeritte yaw) scratchpad'den repoya almak; değerlendirme aracı olarak.
- Dışa aktarma betiği: her kare için görüntü + en yakın EKF2 pozu + yönelim + gimbal + gerçek poz → dosyalar.
- Düz köy dünyası (aynı coğrafi koordinatlar; dokulu zemin, insan + araç) → kayıt biçimi ölçümü.
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
