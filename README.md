# Computer Vision pipeline for Husky bot

## Description
В данном репозитории находится необходимый код для запуска ROS узлов для задач компьютерного зрения в пайплайне. В частности, здесь находятся узлы для:

- публикация списка категорий и их номеров из плана LLM
- сегментации с помощью YoloV8 (только кубики)
- сегментации с помощью OpenSeeD
- 2D-Трекинга объектов
- визуализации облаков точек всех объектов
- извлечения облака точек объекта
- 3D трекинга объекта
- определения 6DoF позы объекта с использованием 3D моделей объектов


## Prerequisites

Необходимо, чтобы на компьютере был установлен ROS. Создать рабочее пространство:
```bash
mkdir husky_tidy_bot_cv_ws
```
В нём создать папку ```src``` и туда поместить данный репозиторий, а так же репозитории с пакетами, которые необходимы для работы данного репозитория, в частности:
- [openseed_src](https://git.sberrobots.ru/mipt.navigation/object_segmentation/openseed-src)
- [communication_msg](https://git.sberrobots.ru/mipt.navigation/interaction/communication_msgs)
## Installation

Тут описана процедура установке на Husky bot для сегментации с помощью OpenSeeD в варианте, где нет conda, но есть virtual env для python. Если установка происходит на компьютере, где есть conda, то проще использовать conda-окружение.
- Установить Python 3.8
```bash
sudo apt-get install python3.8-dev
```
- Создать виртуальное окружение
```bash
virtualenv -p /usr/bin/python3 openseed_env
source openseed_env/bin/activate

```
- Установить pytorch и другие зависимости. Файл requirements.txt находится в [openseed_src](https://git.sberrobots.ru/mipt.navigation/object_segmentation/openseed_src)
```bash
pip3 install torch==1.13.1 torchvision==0.14.1 --extra-index-url https://download.pytorch.org/whl/cu113
python -m pip install 'git+https://github.com/MaureenZOU/detectron2-xyz.git'
pip install git+https://github.com/cocodataset/panopticapi.git
python -m pip install -r requirements.txt
pip install albumentations
pip install Pillow==9.5.0 
pip install wandb
git clone https://github.com/andrey1908/kas_utils.git
cd kas_utils/python
pip install .
```

- Cкомпилировать кастомные ядра CUDA:

```bash
cd /home/administrator/zemskova_ts/husky_tidy_bot_cv_ws/src/openseed_src/openseed/body/encoder/ops
sh make.sh
cd /home/administrator/zemskova_ts/husky_tidy_bot_cv_ws/
```

- Install [rospkg](https://anaconda.org/conda-forge/rospkg) to use ROS inside the conda environment
```bash
conda install -c conda-forge rospkg
```

- Дополнительные зависимости

Для визуализации point clouds

```bash
pip install rosnumpy
```
Для 2D трекинга
```bash
git clone git@git.sberrobots.ru:mipt.navigation/object_segmentation/bot-sort.git
cd BoT-SORT
pip3 install -r requirements.txt
python3 setup.py develop
# Cython-bbox
pip3 install cython_bbox

# faiss cpu / gpu
pip3 install faiss-cpu
pip3 install faiss-gpu
```

Для 3D трекинга
```bash
pip install open3d
pip install transforms3d
```
- Собрать рабочее пространство:

```bash
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.8m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8m.so
```
- Скачать веса модели OpenSeeD. Ссылка на папку с весами модели: [https://disk.yandex.ru/d/SnmwjHCEtSuLzA](https://disk.yandex.ru/d/SnmwjHCEtSuLzA).

model_0003599_demo.pth - веса модели OpenSeeD, дообученной без замороженных слоев последовательно на наборах данных  $LLMObjectSorter_{train} v0$, $LLMObjectSorter_{train} v2$ без доп. аугментаций.

model_full_aug.pth - веса модели OpenSeeD, дообученной c замороженными текстовым кодировщиком и backbone, на наборах данных  $LLMObjectSorter_{train} v3$ с дополнительными аугментациями.

Подробности см. в репозитории [openseed-src](https://git.sberrobots.ru/mipt.navigation/object_segmentation/openseed-src).

- Именить путь к весам и к конфигурационному файлу  [referring_segmentation_ros1/scripts/openseed_node.py](https://git.sberrobots.ru/mipt.navigation/object_segmentation/referring_segmentation_ros1/-/blob/master/scripts/openseed_nide.py)

- Именить путь к исходному коду OpenSeeD внутри модуля OpenSeeD: [referring_segmentation_ros1/scripts/openseed_model.py](https://git.sberrobots.ru/mipt.navigation/object_segmentation/referring_segmentation_ros1/-/blob/master/scripts/openseed_model.py)
## Usage

Узлы используют информацию, публикующуюся в топиках друг друга. Прежде чем использовать узел, необходимо сделать ```source devel/setup.bash``` из папки с рабочим пространством, например
```bash
cd ~/zemskova_ts/husky_tidy_bot_cv_ws
source devel/setup.bash
```

1. Публикация списка категорий и их номеров из плана LLM

```
python3 text_query_generation_server.py
```

Категории объектов публикуются после выполнения действия OpenSeeDSetter от behaviour tree. При первом запуске узел OpenSeeD работает с фиксированными категориями ROSBAG_CATEGORIES ниже (при работе сервера text_query_generation_server.py) После чего в топике /segmentation_labels публикуется сообщение вида:

int32[] classes_ids
string[] labels # names with features
string[] labels_only_cats # names without features

Classes ids идут в том же порядке, что и им соответствующие labels (и labels_only_cats). Обратите внимание, что classes ids идут не упорядочено.

Для эмулирования отправки сообщений от behaviour tree можно использовать, изменив object и location в файле text_query_generation_client.py

```bash
python3 text_query_generation_client.py
```

По умолчанию при запуске OpenSeed node он сегментирует известные категории из fixed_categories.py и ничего не публикует в топике /segmentation_labels.

Новые категории добавляются в конец списка фиксированных, который остаётся неизменным во время всей работы программы. Категория unspecified не добавляется в список категорий для распознавания. Если в плане от LLM будет "orange cube", то это будет рассматриваться как новая категория.
**Логика добавления новых категорий при сегментации**

Новые категории добавляются в конец списка фиксированных, который остаётся неизменным во время всей работы программы. Категория ```unspecified``` не добавляется в список категорий для распознавания. Если в плане от LLM будет ```orange cube```, то это будет рассматриваться как новая категория.


**Предобработка текстовых запросов.**

Запрос \textit{"objects"} (и только он) добавляет в список объектов для сегментации все объекты из фиксированного списка.

Каждая задача в плане, сгенерированным LLM, состоит из полей type(тип), object(объект), location(местоположение). Список текстовых запросов формируется на основе списка категорий в полях object и location для всего списка задач из плана LLM. Для некоторых типов location происходит предобработка текста задачи таким образом, чтобы элементы пайплайна, работающие с фиксированными названиями location могли работать. 

В частности, если location состоит из одного из четырёх слов: "box", "bin", "container", "basket", то название location в плане заменяется на название контейнера в списке фиксированных категорий, а если location состоит из одного из слов, обозначающих тумбочку на английском языке ("nightstand", "bedside table", "cabinet", "basket", "bedside-table"), то оно заменяется на название тумбочки в списке фиксированных категорий. Если в списке location есть слово, обозначающее контейнер, то происходит попытка отнесения по близости текстовых эмбеддингов названия этой категории к категориям известных местоположений контейнеров (например, "red box" -> "orange box"). Если название категории location не подходит ни к одной из известных позиций location, то сервер возвращает состояние FAILED дереву поведения.

ТекстGenerationServer публикует информацию о статусе location в ROS-топик ```location_status```. В случае успеха нахождений названия категорий location в списке известных location, публикуется сообщение "OK!", в случае отсутсвия объекта в списке известных location, публикуется сообщение "Failed to find a location in LLM plan!".

2. Сегментация с помощью OpenSeeD

Проверить, что был запущен ```text_query_generation.py``` и идёт публикация ```/segmentation_labels```.

В другом окна терминале:
```bash
python3 openseed_node.py -vis
```

```Classes_ids``` идут в том же порядке, что и им соответствующие ```labels``` (и ```labels_only_cats```). Обратите внимание, что ```classes_ids``` идут не упорядочено.

Для эмулирования отправки сообщений от дерева поведения можно использовать, изменив ```object``` и ```location``` в файле ```text_query_generation_client.py```.

```bash
python3 text_query_generation_client.py
```

-vis - для визуализации, можно запустить без неё

Визуализация сегментации будет публиковаться в ROS-топике ```/segmentation_vis```.
В ROS-топике ```/segmentation``` публикуются сообщения ```Objects.msg``` вида:

```bash
Header header

int32 num

float64[] scores
int32[] classes_ids
int32[] tracking_ids 
Box[] boxes
Mask[] masks
```


3. Запуск визуализации облаков точек всех объектов

```bash
DEPTH_TO_POINT_CLOUD_ALLOW_PYTHON_IMPLEMENTATION=1 python3 visualize_objects_point_cloud.py
```

Для всех последующих узлов нужны результаты трекинга (2D трекинг использует 3D трекинг)

4. 2D-tracking

python3 bot_sort_node.py -vis

На этом этапе объектам назначились ID, которые могут быть использованы как для 3D трекинга, так и для формирования Point Cloud 
Выходной топик для визуализации /tracking_vis - изображения с объектами трекинга.
Топик без визуализации /tracking.

5. a) Формирование Point Cloud

```bash
DEPTH_TO_POINT_CLOUD_ALLOW_PYTHON_IMPLEMENTATION=1 python3 object_point_cloud_extraction_node.py -vis
```

Для тестирования формирования Point Cloud можно использовать сервис /object_point_cloud_extraction/set_object_id

Object id можно посмотреть в топике 2D трекинга. Например,

```bash
rosservice call /object_point_cloud_extraction/set_object_id 3
```
При этом Point Cloud будет формироваться только на основе результатов сегментации для этого object_id на кадрах, которые приходят после вызова этого сервиса.

б) 3D tracking

```bash
python3 tracker_3d_node.py -vis
```

Выходной топик для визуализации /tracked_objects_3d_vis - массив маркеров (MarkerArray) с объектами.

```bash
python3 tracker_3d_node.py -vis
```

6. Оценка 6DoF позы

Для выбора объекта нужно сделать:

```bash
python3 object_pose_estimation_node.py -vis
```

```bash
rosservice call /object_pose_estimation/get_object_pose <n_traking_id>
```
## Project status
v0.

## Типичные ошибки

1. При запуске узла OpenSeeD вылетает ошибка:
```bash
Traceback (most recent call last):
File "openseed_node.py", line 109, in <module> segmentation_node.start()
File "openseed_node.py", line 52, in
start self.image_sub = rospy.Subscriber( 
File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/topics.py", 
line 563, in
__init__ super(Subscriber, self).__init__(name, data_class, Registration.SUB)
File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/topics.py", line 140, 
in __init__ raise ValueError("topic parameter 'data_class' is not initialized")
ValueError: topic parameter 'data_class' is not initialized 
```

Причина: не публикуются сообщения из ROS-топика камеры.

2. При запуске узла OpenSeeD вылетает ошибка:
```bash
Traceback (most recent call last): 
File "openseed_node.py", line 109, in <module> segmentation_node.start() 
File "openseed_node.py", line 52, in 
start self.cats_sub = rospy.Subscriber( 
File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/topics.py", 
line 563, in 
__init__ super(Subscriber, self).__init__(name, data_class, Registration.SUB)
File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/topics.py", line 140,
in __init__ raise ValueError("topic parameter 'data_class' is not initialized")
ValueError: topic parameter 'data_class' is not initialized
```

Причина: не запущен узел ```text_query_generation_server.py```, см. 1. Публикация списка категорий и их номеров из плана LLM.

## Known issues

1. Для модели model_full_aug.pth из-за замороженного текстового кодировщика запрос "bin box basket container" оказался слишком длинный для того, чтобы отдельные слова воспринимались для сегментации, поэтому запросы типа "white box" не работают для данной версии весов. Запросы типа "red cube", "green cube" работают, как ожидается.