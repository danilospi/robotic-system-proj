**Indice**

<!-- TOC -->

- [Introduzione](#introduzione)
- [Testo](#testo)
- [Avviare il Progetto](#avviare-il-progetto)
  - [Prerequisiti](#prerequisiti)
  - [Eseguire il punto 1](#eseguire-il-punto-1)
  - [Eseguire il punto 2](#eseguire-il-punto-2)
  - [Eseguire il punto 3](#eseguire-il-punto-3)

<!-- /TOC -->

# Introduzione
Questa documentazione descrive brevemente il progetto implementato.<br> Tutto il codice è disponibile su GitHub al seguente link: `https://github.com/danilospi/robotic-system-proj`. [(Link Diretto)](https://github.com/danilospi/robotic-system-proj)<br>
Nella prima parte di documentazione è presente il testo da cui nasce il progetto, mentre nella seconda parte viene spiegato passo passo come verificare i vari punti del progetto.

# Testo

__(1) Modello__

Si effettui la simulazione grafica di un robot mobile a due ruote indipendenti, implementando il controllo
in velocita delle ruote, l’odometria, il controllo polare del moto.
Per il robot, si consideri ogni ruota modellata come un sistema `massa con attrito` utilizzando i seguenti parametri:

- Massa del robot: *20 kg( 10 kg per ruota)*
- Coefficiente di attrito viscoso: *7 · 10^(-5)*
- Forza di spinta massima per ruota: *130 N*
- Velocita massima: *2 m/s*
- Accelerazione/decelerazione: *a piacere*
  
Si producano i grafici di velocita e posizione, in modo da dimostrare la corretta taratura dei controllori.
Si consideri un ambiente bidimensionale, di dimensioni scelte a piacere, popolato da ostacoli fissi e da oggetti da catturare (vedi sezioni successive).

__(2) Path Planning__

Quale pianificatore del percorso si utilizzi l’algoritmo di cell decomposition da implementare in PHIDIAS o in Python a piacimento.

__(3) Strategia__

Si implementi un programma in PHIDIAS che consenta le seguenti funzionalita. Si consideri la presenza di oggetti di forma circolare (dischi o cilindri), di colori rosso,verde e blu, che il robot deve catturare e posizionare, a seconda del colore, in aree ben specifiche. Si scelgano 20 posizioni prefissate e, tramite un’apposita procedura PHIDIAS, si posizionino casualmente 10 dischi nelle 20 posizioni; si generino casualmente anche i colori. Si supponga che il robot sia dotato anteriormente di un sensore di colore in grado di determinare anche se il disco è assente.
In tale ambiente, a seguito dell’invocazione di un’opportuna procedura PHIDIAS, il robot deve:

1. effettuare la scansione delle 20 posizioni allo scopo di determinare presenza e colore del disco, usando sempre il criterio della posizione piu vicina.
2. effettuare la cattura ed il rilascio dei 10 dischi usando una dando priorità al rosso, poi al verde e infine al blu.

# Avviare il Progetto

In questa sezione vengono descritte le procedure per poter avviare il progetto e verificare l'esecuzione dei 3 punti da cui è costituito.

## Prerequisiti

I prerequisiti per eseguire il progetto sono:

1. Python: v3.11
2. PyQt5: v5.15.9
3. scipy: v1.10.1
4. matplotlib: v3.7.1
5. numpy: v1.24.2
6. requests: v2.28.2
7. pygame: v2.3.0
8. PyOpenGL: v3.1.6
9. pyserial: v3.5

## Eseguire il punto 1

Per poter eseguire il punto 1 occorre seguire la seguente procedura:
- Aprire un terminale.
- Spostarsi sulla directory del progetto (`robotic-system-proj`).
- Eseguire con python il seguente script: `velocity_position_plot.py`
- Questo script farà partire una simulazione di 3 secondi al termine della quale verranno generati 3 grafici per verificare la corretta taratura dei controllori. 

I 3 grafici sono:
  1. Velocità della ruota sinistra.
  2. Velocità della ruota destra.
  3. Posizione del robot.

![vl-plot](/img/vl_plot.png)
<p style="text-align: center;">[Figura 3.2.1]: Velocità Ruota Sinistra</p>

![vr-plot](/img/vr_plot.png)
<p style="text-align: center;">[Figura 3.2.2]: Velocità Ruota Destra</p>

![position-plot](/img/position_plot.png)
<p style="text-align: center;">[Figura 3.2.3]: Posizione Del Robot</p>

Per poter ottenere questi risultati sono stati scelti, per le 2 ruote, 2 controllori di tipo proporzionale integrale con saturazione. I valori scelti per le variabili sono: kp = 18.0, ki = 2.0

## Eseguire il punto 2

Per poter eseguire il punto 2 occorre seguire la seguente procedura:
- Aprire un terminale.
- Spostarsi sulla directory del progetto (`robotic-system-proj`).
- Eseguire con python il seguente script: `gui.py`
- Questo script farà partire una simulazione nella quale saranno presenti 3 ostacoli e tramite le loro coordinate  e l'algoritmo di cell decomposition verrà generato un grafo sul quale si muoverà il robot. Grazie a questo algoritmo il robot sarà in grado di navigare l'ambiente senza sbattere contro gli ostacoli.

![cell-decomposition-graph](/img/cell_decomposition_graph.png)
<p style="text-align: center;">[Figura 3.2.3]: Grafo Generato</p>

Tutto il codice che consente di analizzare la dimensione dell'ambiente ed i relativi ostacoli in modo tale da generare il grafo sopra è presente al seguente script: `cell_decomposition.py`

## Eseguire il punto 3

Per poter eseguire il punto 3 occorre seguire la seguente procedura:
- Aprire un terminale.
- Spostarsi sulla directory del progetto (`robotic-system-proj`).
- Eseguire con python il seguente script: `gui.py`
- Aprire un secondo terminale.
- Spostarsi sulla directory del progetto (`robotic-system-proj`).
- Eseguire con python il seguente script: `strategy.py`
- Faccio ciò da un lato si avrà la simulazione attiva e dall'altro un terminale dal quale lanciare i comandi per far muovere il robot.

I comandi disponibili sono i seguenti:
- `gen_color()`: per generare 10 dischi colorati.
- `scan()`: per far scansionare al robot le 20 posizioni.
- `pick()`: per far raccogliere al robot i dischi colorati.
- `reset_all()`: per resettare la base di conoscenza del robot.

Per una corretta esecuzione occorre eseguire i comandi nel seguente ordine:
1. `gen_color()`
2. `scan()`
3. `pick()`

![Alt Text](img/point3.gif)
<p style="text-align: center;">[Figura 3.4.1]: Punto 3 In Esecuzione</p>