# laser-rhythmus-apparat (lra)

## Vorraussetzungen

- PortAudio

## Bauteile

- 1x Mikrokontroller (Arduino UNO R4 Minima):
https://store.arduino.cc/collections/boards-modules/products/uno-r4-minima

- 5x Lasersensoren (VL53L0X):
https://www.amazon.de/AZDelivery-VL53L0X-Flight-kompatibel-Arduino/dp/B086TSNTFD/ref=pd_rhf_dp_s_pd_crcd_d_sccl_1_1/260-7274141-1823459?pd_rd_w=MJp5l&content-id=amzn1.sym.d36cdb68-efc9-4a4e-bcda-d349636a649b&pf_rd_p=d36cdb68-efc9-4a4e-bcda-d349636a649b&pf_rd_r=F14WJGG2JM0JBDSHKK67&pd_rd_wg=vXWgg&pd_rd_r=3aa74498-f5a2-4ef4-a847-166cdc81a463&pd_rd_i=B086TSNTFD&th=1

- Kabel (weiblich-weiblich, weiblich-männlich, männlich-männlich je 40x):
https://www.amazon.de/Female-Female-Male-Female-Male-Male-Steckbr%C3%BCcken-Drahtbr%C3%BCcken-bunt/dp/B01EV70C78/ref=pd_sbs_d_sccl_3_1/260-7274141-1823459?pd_rd_w=ufcsj&content-id=amzn1.sym.6ebe1bf9-1e45-40dc-b013-9e8c7ce1588f&pf_rd_p=6ebe1bf9-1e45-40dc-b013-9e8c7ce1588f&pf_rd_r=QET6FYKJAAS4DGCPHY3Z&pd_rd_wg=5ZMju&pd_rd_r=534dfbd1-4f86-4f5e-9f3f-5222407d3539&pd_rd_i=B01EV70C78&th=1

- 2x Steckbretter:
https://www.amazon.de/VoltMate-Breadboard-Steckbrett-Kontakten-Kompatibel/dp/B0CX8MMN6T/ref=sr_1_19?__mk_de_DE=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=2DG1O0GSYJMBC&dib=eyJ2IjoiMSJ9.isoB5-f5De9M0yk8l0f_ZqVxnxuJLnuHbEY37w30nFAzLCG1BWdmRTWUOCtWYLrVX6vD-GnijEXHgpSpfZp1AHaFIJ6JCzyp45WKnrGM2GAYzSpCblMe2oBLJfu9copTzN-wt-mOOb5NM10R8HBa5RVTCcadpzdtBhGT8MlUchusWxu4pR1A5mrO3FfFGSOObREQFpRHw02UBOIW1EmOoABw9hobIsTmVmhUqU4twpj9Tim8L1n0N6FoT-4eK84hBCN4QNXvkP5A-EA9iexjcMMwgGvdSm9DDb3Tl5GLjXY.dS3FpZTQO1DpBbMb3-qmpAPZMR60G3YBakS3LwTY8jY&dib_tag=se&keywords=steckbrett&qid=1714147855&s=industrial&sprefix=steckbrett%2Cindustrial%2C193&sr=1-19&th=1

## Arduino

Falls ein Fehler auf Linux ubuntu auftreten sollte:

https://support.arduino.cc/hc/en-us/articles/9005041052444-Fix-udev-rules-on-Linux#renesas

Wie angegeben die post_install.sh Datei von GitHub herunterladen und im Verzeichnis der Datei das Terminal öffnen.

Dem .sh Skript Ausführungsrechte erteilen

```
chmod +x ./post_install.sh
```

Die Datei ausführen

```
sudo ./post_install.sh
```