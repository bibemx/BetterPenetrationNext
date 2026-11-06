# Проект является полностью завайбкоженным и этот README в том числе - можете увидеть главную причину подорожанния оперативки
# BetterPenetrationNext 1.0.9 — KKS

[English](README_eng.md) · [Скачать релиз](https://github.com/bibemx/BetterPenetrationNext/releases)

Реактивный риг и локальная деформация анимированных мешей для Koikatsu Sunshine / Unity 2019.4

## Возможности

- Режим **BPN** с Adaptive Mesh Landmarks, ориентированной входной плоскостью R0 и направляющей канала.
- Закреплённое основание Source, сохранение длин секций и ручное переопределение длины и ширины уже при загрузке сцены.
- Независимое раскрытие секторов Receiver по конечным срезам реальной открытой оболочки Source. Кэшированное дерево поиска перестраивает границы, не топологию; разные веса швов сохраняются.
- Плавный подвод к конечному ориентиру.
- Раскрытие входа с сохранением привязок, симметрией процедурных смещений и восстановлением после прекращения контакта. Проверенное время восстановления — 4,5 с; это параметр визуализации.
- X-Ray с исходными текстурами, раздельным отображением Source/Receiver и дополнительным точечным индикатором. Исходные материалы не перезаписываются.
- Режим **BP** использует сохранённый контроллер BetterPenetration внутри этой DLL. Отдельная оригинальная DLL одновременно не нужна.

## Требования

| Компонент | Требование |
|---|---|
| Игра | Koikatsu Sunshine, Unity **2019.4.9f1**, Windows x64 |
| Загрузчик | [BepInEx 5.x](https://github.com/BepInEx/BepInEx/releases), включая `0Harmony.dll` |
| API | [IllusionModdingAPI / KKSAPI](https://github.com/IllusionMods/IllusionModdingAPI/releases) |
| ZIPMOD | Sideloader из [BepisPlugins](https://github.com/IllusionMods/BepisPlugins/releases) |
| Выбор модели | [UncensorSelector](https://github.com/IllusionMods/KK_Plugins/releases), версия плагина не ниже **3.11.1** |
| Кости моделей | [ModBoneImplantor](https://github.com/IllusionMods/ModBoneImplantor/releases), версия плагина не ниже **1.1.1** |
| Source | Проверен `[KKS][Penis][BP] SoS.zipmod`, GUID `uncensor.BP_SoS` / выбор `BP SoS` |
| Receiver | `BPN.Reciever.Profundis.zipmod` из этого выпуска |

Зависимости игры, оригинальный SoS и сторонние плагины не включены в архив. Исходные совместимые модели доступны через [upstream BetterPenetration](https://github.com/IllusionMods/BetterPenetration/releases). Поддержка Studio и других игр в этом выпуске не заявляется. ConfigurationManager необязателен: у BPN есть своё окно настроек.

## Установка

1. Установите зависимости и Source SoS. Закройте KKS и Studio.
2. Скачайте **`BetterPenetrationNext-1.0.9-KKS.zip`** из Releases и распакуйте в отдельную папку. Автоматический GitHub `Source code.zip` установочным пакетом не является.
3. Запустите `Install.cmd` и укажите папку с `KoikatsuSunshine.exe`. Или выполните:

   ```powershell
   powershell -NoProfile -ExecutionPolicy Bypass -File .\Install.ps1 -GameRoot "D:\Games\KKS"
   ```

4. Установщик проверяет пакет, сохраняет затрагиваемые файлы, устанавливает DLL и Receiver, применяет проверенный профиль. Старый Receiver `BetterPenetrationNext-1.0.zipmod` и найденные оригинальные/Enhanced DLL отключаются переименованием с резервной копией. Уже отключённые файлы остаются на месте; обновление с тестового BPN поддерживается.
5. В UncensorSelector выберите **BetterPenetration Next 1.0** для Receiver и **BP SoS** для Source. Не оставляйте другие активные копии Receiver с GUID `org.betterpenetration.next.body.v1`.
6. Загрузите сцену. **B** включает X-Ray, **N** открывает настройки. Эти клавиши можно переназначить в разделе `Next.Controls`.

Установщик не меняет карточки, пользовательские текстуры и Source ZIPMOD. Не копируйте DLL поверх запущенной игры. После установки исходный BP доступен через переключатель **BPN / BP**; одновременно запускать оригинальную DLL нельзя.

## Настройки

Профиль `release-defaults.json` входит в установщик и встроен в DLL. Поэтому новая конфигурация и удаление старого `BepInEx/config/org.betterpenetration.next.cfg` дают одинаковые значения по умолчанию. Удаляйте конфиг только при закрытой игре и после собственной резервной копии.

| Проверенная настройка | Значение |
|---|---:|
| SourceLengthScale | 0.7727273 |
| SourceWidthScale | 1 |
| CanalLengthScale | 1.384297 |
| ContactDamping | 5.419422 |
| CervixMaxDisplacement | 0.08 |
| CervixFollowStrength | 1 |
| RecoveryTime | 4.5 |
| X-Ray / окно | B / N |

Размеры применяются относительно анимированной модели из карточки. Эти числа проверены на конкретной паре моделей и не являются универсальными. В текущем окне показаны настройки реактивного рига; внутренние совместимые параметры старых алгоритмов сохранены в коде, но скрыты из основного интерфейса. Отдельный `PathFile` для подготовленного Receiver не требуется.

## Откат

Закройте игру и запустите `Rollback.ps1` **из папки резервной копии**, путь к которой напечатал установщик:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "D:\Games\KKS\BepInEx\BetterPenetrationNextBackups\1.0.9-...\Rollback.ps1"
```

Восстанавливаются прежние DLL, Receiver и конфигурация, оригинальные активные файлы возвращаются из отключённого состояния. Изменённые после установки файлы дополнительно сохраняются перед откатом. Резервные копии не удаляются.

## Сборка и тесты

Нужен .NET SDK с поддержкой проектов `net6.0` и `net46` (использован SDK **6.0.402**) и локальная установка KKS с зависимостями. Сборка скачивает официальный пакет reference assemblies .NET 4.6; игровые DLL в репозиторий не входят.

```powershell
dotnet build src/Next.Plugin -c Release -p:GameRoot="D:\Games\KKS" -m:1
dotnet run --project tests/Next.Core.Tests -c Release
```

Результат: `src/Next.Plugin/bin/Release/net46/BetterPenetrationNext.dll`. Численные тесты запускаются без игры. В `tests/AnatomicalRuntimeReplay` сохранён инструмент воспроизведения; личные дампы сцен и исходные внешние модели в Git не публикуются. Предварительно собранный shader bundle находится в `build/shader`; исходник шейдера — в `UnityLab/Assets/Shaders`.

В выпуске пройдены 173 численных теста, нативные проверки реальных мешей в Unity и пользовательская проверка активной сцены. Проверка одной сцены не гарантирует отсутствие пересечений на любых карточках и анимациях. Показатели времени полного кадра зависят от сцены и фоновой нагрузки; гарантированное повышение FPS не заявляется. Краткая область проверки: [docs/validation.md](docs/validation.md).

## Лицензия

Код — [GPL-3.0-or-later](LICENSE). Авторы исходного BetterPenetration и моделей указаны в [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) и манифестах ZIPMOD. Лицензия кода не заменяет условия сторонних игровых ассетов.
