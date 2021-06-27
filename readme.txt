DAS Clock
- STLink 1.pinje nincs +3V3-ra kötve
- Táp IC-k ENgedélyező pinje nincsenek bekövtve... 
- "Teljesen fölösleges részek pl reset áramkör" van beterveze (táp LED helyett)
- nincs táp LED
- Komponens mérékegysgék uH uF kerverdése
- Lehet nem/teljesen kompatibilis a homéro IC-a tápjával, erre nagyon kell figyelni a BOM készitesekor!
- Nem túl szerencsés dizájn az UMAIN mérése...
- uC csere Flash korlát miatt STM32F103C4T6-ről STM32103C8T6-ra
- a proci túl kicsit nagyobbat kellett volna választani
- A prociba lebegőpontos számok nefértek de kiderült hogy parancsértelmező sem fog.
- Lehetne több státusz LED a panelen. (jó lenne ha lenne 1 LiveLED,2xLOCK led és 2db LED az MV205-nek és 1db tápled)
- Vannak olyan alkatérszek amiket át kell kötni pl az AMP6...
- A padek nagyon picik megbizható kézi ültetéshez, gépire Gábor szerint nem alkalmas
- Az MV205-ök vezérlése 0-ra akit ezt jelölni kell a kapcsolási rajzon
- A tápokat erősen hűteni kell
Todo:
- Nagyobacsaka offeszethibák is vannak az analóg mérésekben, még belefér?


- Panel tápfeszültsége kb: 14V-15V, Áramfelvétele:

power up szekvencia:
Az MV341-es folyamatosan füt. Amikor az MV341-es fütőárama 300mA-alá csökken X-idő után
az MV205_1-es kezd fűteni majd amikor 200mA alá csökken a fűtőárama az MV205_2-es kezd fűtésbe.

- Nincsenek hiba állapotok megvalósitva, csak diagnosztika és power up szekvencia.
- A homérésékeltének tájkoztató jellegu .

LED-ek:
PB7: LiveLED
PB6: Lock1
PB5: Lock2
PB4: MV205_1 & MV205_2 Enabled



-10MHz bemérése
- 22.57
- A 74SN1G04DBVR a 14-ek helyett
- hurokszűrőben: C78 22uF helyett 10uF és R69: 150K és 510-re cseréltük 
- BF799-es hiányzott
- Hangolófeszültség:
- 22.7592: 2.53
- 24.576MHz: 2.51V

- R18: 47K-> végtelen nem kell a csillipitás
- R25: 270OHM -> végtelen
- R21: 18R0 -> 0R 
- R16: 270-> végetelen
- AMP1 ERA SM1+ helyett ERA SM2+
- R153:10K -> 8.2K 
- R104: 10K-> 8.2K
- R52: 10K -> 9.375K  10K||150K

- Forditott kondik voltak
- A szűrök polaritása



