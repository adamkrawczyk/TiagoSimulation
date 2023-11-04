# Co dalej: 

- opis instniejących symulatorów i opis, że celem tej pracy jest dostosowanie jednego symulatora do działania z robotem tiago i porównanie szczególnie z Gazebo
- naszym celem jest efektywna symulacja czyli w odpowiedniej szybkości i jakości (ja proponuję rozwiązanie w o3de i je waliduje). Ocena ilościowa jest o tyle fajna, że możemy względnie gazebo porównać np. prędkość działania. Wskaźnik walidacji to szybkość działania względem o3de. 

Właściwe będzie porównanie szybkościowe i jakościowe. Raczej nie bedziemy robić jakichś skomplikowanych scenariuszy.

Dalej innymi problemami jest kompatybilnośc z innymi narzędziami. 

Projektujemy system w mojej wersji czyli o3de skladający się z czesci wspolnej i mojej czesci. Coś takiego samego staramy się pozniej zrobić w gazebo i robimy eksperyment. (takie mapy trasy i wgl...)



### Dlaczego isaac sim jest gorszy od o3de, czemu wybralismy o3de. 

Isaac sim jest bardzo podobny do o3de, jest bardziej zaawansowany ale płatny. Wykorzystuje physx 5 tak samo jak o3de. Do generowania grafiki używany jest unreal engine vs o3de atom renderer. Porównywanie obu symulatorów jest o tyle trudne, że o3de wciąż jest w fazie rozwoju. 

### Sprawdzenie ile trzeba zmienic zeby dostosować do o3de (uzywany jest nav1 vs nav2)

### Termin wykonania od, do: 



## O3DE vs Isaac Sim

| Item                | O3DE                 | Isaac Sim         |
| ------------------- | -------------------- | ----------------- |
| Code standard       | Open source          | Closed source     |
| PhysX version       | 5                    | 5                 |
| Engine              | O3DE                 | Unreal Engine     |
| Renderer            | Atom                 | ? (RTX Omniverse) |
| ROS2 support        | Yes (without bridge) | Yes (with bridge) |
| ROS1 support        | No                   | Yes (with bridge) |
| Multi-GPU support   | No                   | Yes               |
| Multi-robot support | Yes                  | Yes               |

## Typical O3DE issues

- Still under development and somethimes unstable
- PhysX 5 is just released and some bugs might occur
- Weights of the robot are not properly set (not sure if this is also for the Isaac Sim - should be sice both runs on PhysX 5)
- Some debug buses are inefficient and slow down the simulation (e.g. back-and-forth communication)


## Typical Isaac Sim issues

- Closed source
- Unreal Engine is not open source
- Paid
- ROS support is with bridge which slows down the simulation

## O3DE Pros

- Open source
- Increasing popularity
- Well documented

## Isaac Sim Pros

- More advanced and stable
- More features
- More realistic graphics (Easier to achieve in Unreal Engine)
- Well documented