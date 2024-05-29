
# cam_publisher

## Overview

O pacote `cam_publisher` é um nó ROS desenvolvido para capturar imagens de uma câmera e publicá-las em um tópico ROS. Ele utiliza OpenCV para captura de imagens e `cv_bridge` para converter imagens OpenCV para mensagens ROS. Os parâmetros de calibração da câmera são carregados de um arquivo YAML.

**Keywords:** ROS, OpenCV, Camera, Image Publisher

### Building from Source

#### Dependencies

- Robot Operating System (ROS)
- OpenCV
- YAML-CPP

```sh
sudo rosdep install --from-paths src
```

## Config Files

### Configuração da Câmera

Arquivos de configuração da câmera, como por exemplo (`config/front_camera.yaml`):

```yaml
image_width: 1920
image_height: 1080
camera_matrix:
  rows: 3
  cols: 3
  data: [1332.2905012603742, 0.0, 1001.619419558725, 0.0, 1313.963171950812, 491.9312293181554, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.5176939966889225, 0.17338115766870202, 0.023434654253362403, -0.008230427613718875, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [862.3757981998015, 0.0, 958.1924244615784, 0.0, 0.0, 1187.5335378689128, 503.67812178921946, 0.0, 0.0, 0.0, 1.0, 0.0]
```

## Launch Files

### Launch File Principal

Arquivo de lançamento principal (`launch/cam_publisher.launch`):

```xml
<launch>
  <node name="image_publisher" pkg="cam_publisher" type="camera_node" output="screen">
    <param name="camera_info_yaml" value="$(find cam_publisher)/config/front_camera.yaml"/>
    <param name="device" value="2"/>
    <param name="topic_name" value="camera"/>
  </node>
</launch>
```

## Nodes

### image_publisher

O nó `image_publisher` lê imagens de uma câmera e publica as imagens já em um tópico ROS.

#### Published Topics

- `/camera/image` (`sensor_msgs/Image`): Imagens capturadas pela câmera.

#### Parameters

- `camera_info_yaml` (string): Caminho para o arquivo YAML contendo os parâmetros de calibração da câmera.
- `device` (int): Número do dispositivo da câmera.
- `topic_name` (string): Nome do tópico onde as imagens serão publicadas. Default: "camera".
