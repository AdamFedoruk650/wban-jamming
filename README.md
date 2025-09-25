# wban-jamming
Ten projekt implementuje symulator jammingu w ns3 dla bezprzewodowych urządzeń medycznych działających wewnątrz ciała zgodnie ze standardem IEEE 802.15.6.
Symulator korzysta z zaimplementowanej dla ns3 specyfikacji standardu IEEE 802.15.6, którą można pobrać z https://apps.nsnam.org/app/wban/.
Przy pracy nad symulatorem zmodyfikowane zostały plik wban-propagation-model.cc oraz wban-propagation-model.h zaciągnięte z poprzednio wymienionego projektu.
Symulacje przeprowadzane były w wersji ns-3.45.

W celu uruchomienia symulatora należy:
1) pobrać ns w wersji 3.45
2) pobrać projekt https://gitlab.com/DrishtiOza/wban/-/tree/main?ref_type=heads i wstawić go do katalogu w ns-allinone-3.45\ns-3.45\contrib\wban
3) pobrać symulator wban-jamming.cc i wstawić go w path ns-allinone-3.45\ns-3.45\contrib\wban\examples
4) zbudować i skompilować ns-3.45 ze wstawionymi plikami
5) wejść do katalogu ns-allinone-3.45\ns-3.45\contrib\wban i uruchomić symulator na przykład poleceniem:
./ns3 run "wban-jamming --bodyOrgan=heart-402 --scanTarget=jam --scanCsv=jam-scan.csv --scanStart=-200 --scanStop=200 --scanStep=0.1 --jamPackets=500 --jamThreshold=0.02"
w tym przypadku wybieramy serce, 402 MHz, symulujemy położenia jammera od -200 m do +200 m z dokładnością kroku przemieszczenia 0.1 m, nadajnik i odbiornik są nieruchome.

Do przeprowadzania symulacji powstały dodatkowe skrypty w pythonie które tworzą pliki csv i ploty w zależności od zadanych parametrów symulacji.
