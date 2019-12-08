**Contribuidores/Contributors**:
- [IagoMendes](https://github.com/IagoMendes)
- [ehrhardt98](https://github.com/ehrhardt98)
- [LiuSeeker](https://github.com/LiuSeeker)
- [vitorsv1](https://github.com/vitorsv1)

(PT-BR)

### Projeto 1 - Implementação de funcionalidades de sensores do TurtleBot personalizado

Básico (C):

- - [x] Robô reage de forma diferente a 2 objetos, que são detectados usando técnicas diferentes (por exemplo features e cores).
- - - [x] Um dos comportamentos visuais precisa ser de seguir o objeto.
- - - [x] Não pode usar deteção simples de círculos via Hough.
- - [x] O controle tem que ser feito usando máquinas de estado Smach.
- - [x] Apresenta comportamento de sobrevivência capaz de usar Laser e IMU.

Proficiente (B):

- - [x] Robô reage diferente a estímulo visual de 2 objetos, dos quais um deles precisa ser aprendido na hora.
- - [x] Os dois objetos precisam ser detectados usando técnicas diferentes. 
- - - [x] Não pode usar deteção simples de círculos via Hough.
- - - [x] Um dos comportamentos visuais precisa ser de seguir o objeto, e o comportamento de seguir precisa usar controle proporcional.
- - [x] O controle tem que ser feito usando máquinas de estado Smach.
- - [x] Apresenta comportamento de sobrevivência capaz de usar Laser e IMU.

Avançado (A):

- Mais UM destes:

- - - [ ] Depois de localizado um objeto de interesse, o robô deve retornar o ponto de origem usando uma das opções (1) integração da IMU, (2) SLAM, (3) integração do fluxo óptico, (4) Landmarks ALVAR.
- - - [ ] Aplicar o YOLO e fazer transições da máquina de estados em função das categorais do YOLO.
---

(EN-US)

### Project 1 - Custom TurtleBot Sensor Functionality Implementation

Basic (C):

- - [x] Robot reacts differently to 2 objects, which are detected using different techniques (eg features and colors).
- - - [x] One of the visual behaviors needs to be to follow the object.
- - - [x] You cannot use simple circle detection via Hough.
- - [x] Control has to be done using Smach state machines.
- - [x] Features survival behavior capable of using Laser and IMU.

Proficient (B):

- - [x] Robot reacts differently to visual stimulation of 2 objects, one of which needs to be learned on the spot.
- - [x] Both objects need to be detected using different techniques.
- - - [x] You cannot use simple circle detection via Hough.
- - - [x] One of the visual behaviors needs to be following the object, and the following behavior needs to use proportional control.
- - [x] Control has to be done using Smach state machines..
- - [x] Features survival behavior capable of using Laser and IMU.

Advanced (A):

- One more of these:

- - - [ ] Once an object of interest has been located, the robot must return to its point of origin using one of the options (1) IMU integration, (2) SLAM, (3) optical flow integration, (4) Landmarks ALVAR.
- - - [ ] Apply YOLO and make state machine transitions based on YOLO categories.
