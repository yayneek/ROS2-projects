Tydzień 1 – Start i środowisko

Zainstaluj ROS2 (np. Humble) w Dockerze albo na WSL2 (żeby nie męczyć się z Linuxem od zera).

Uruchom przykładowe pakiety ROS2 (turtlesim, demo_nodes).

Zapoznaj się z podstawowymi narzędziami: ros2 run, ros2 topic echo, ros2 launch.

Cel tygodnia: umiesz uruchomić prosty node, rozumiesz publish/subscribe i uruchamiasz Turtlesima.




Tydzień 2 – Symulacja robota mobilnego

Uruchom TurtleBota3 w Gazebo (turtlebot3_gazebo).

Naucz się sterować robotem ręcznie (teleop_twist_keyboard).

Podejrzyj dane z sensorów (np. ros2 topic echo /scan dla LiDARa).

Cel tygodnia: rozumiesz, jak robot widzi otoczenie i potrafisz sterować nim w symulacji.




Tydzień 3 – Nawigacja i SLAM

Zainstaluj i uruchom pakiet Nav2 (Navigation2 stack).

Naucz robota tworzyć mapę środowiska (SLAM Toolbox).

Zapisz mapę do pliku i uruchom nawigację na tej mapie.

Cel tygodnia: robot sam jedzie do wskazanego punktu w symulacji, wykorzystując LiDAR i mapę.




Tydzień 4 – Automatyzacja zadania

Napisz własny node w Pythonie sterujący robotem, który:

np. odwiedza kilka punktów po kolei (waypoint navigation).

Dodaj prostą logikę (np. robot wraca do punktu startowego).

Cel tygodnia: własny kod steruje ruchem robota zamiast ręcznego klikania w RViz.




Tydzień 5 – Integracja percepcji

Dodaj kamerę (np. plugin do TurtleBota w Gazebo).

Użyj OpenCV w ROS2, żeby rozpoznać prosty obiekt (np. kolorową kulę).

Spróbuj, żeby robot jechał do obiektu widocznego w kamerze.

Cel tygodnia: masz połączenie percepcji z decyzjami sterującymi.




Tydzień 6 – Projekt „demo”

Połącz wszystko w jeden scenariusz:

Robot startuje w punkcie A.

Buduje mapę albo ładuje istniejącą.

Jedzie do wskazanego punktu.

Wykrywa obiekt i podjeżdża bliżej.

Wraca do bazy.

Cel tygodnia: masz gotowy pipeline, który możesz zaprezentować.




Tydzień 7 – Dokumentacja i prezentacja

Przygotuj dokumentację (repozytorium GitHub/GitLab z kodem, opis kroków).

Nagraj krótkie demo wideo z działania robota w symulacji.

Zrób slajdy z architekturą systemu (diagram node’ów ROS2, opis działania).

Cel tygodnia: możesz pokazać projekt pracodawcy jako przykład kompleksowego podejścia.
