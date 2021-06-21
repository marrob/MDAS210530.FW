DAS Clock problémák
- STLink 1.pinje nincs +3V3-ra kötve
- Táp IC-k nincsenek engedélyezve
- "Teljesen fölösleges részek pl reset áramkör" van beterveze
- nincs táp LED
- Komponens mérékegysgéek
- Lehet nem kompatibilis a homéro IC-a tápjával?
- nem túl szerencsés dizájn az UMAIN mérése...
- a float-os printf lib nem fér bele uC-be, vagy cserélünk vagy hexában kommunikálunk
- nagyobacsaka offeszethibák is vannak az analóg mérésekben
- a proci túl kicsit nagyobbat kellett volna választani
- A prociba lebegőpontos számok nefértek de kiderült hogy parancsértelmező sem fog.


- Panel tápfeszültsége kb:14V-15V
power up szekvencia:
- MV341 - nagy áramot vesz fel mert melegszik, 
  ez egyböl indul, de ha küls órajel van (Int/Ext).
  akkor nem kell bevárni a külso IntExt.
  hiba állapotok megvalósítás.
- 300mA lecsökken: ezt meg kell majd nézni 100mA-körül már biztos befüteni
- MV205 - 200mA/80mA és csökken
....
A homérésékeltének tájkoztató jellegu 

A Lockok csak bemnetek... pozotiv logikában...