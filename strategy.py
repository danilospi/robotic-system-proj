import sys

sys.path.insert(0, "/Users/Danilo/Documents/Sistemi Robotici/phidias/lib")

from phidias.Types import *
from phidias.Main import *
from phidias.Lib import *
from phidias.Agent import *

class block(Belief): pass

class scan_nearest_position(Belief): pass

class go_to_nearest_block(Belief): pass

class go_to_bucket(Belief): pass

class sense_color(Belief): pass

class generate_blocks(Belief): pass

class go_to_start(Belief): pass

class scan(Procedure): pass

class get_vertex(Procedure): pass

class get_red(Procedure): pass

class get_green(Procedure): pass

class get_blue(Procedure): pass

class reset(Procedure): pass

class reset_red(Procedure): pass

class reset_green(Procedure): pass

class reset_blue(Procedure): pass

class sense(Procedure): pass

class pick(Procedure): pass

class pick_red(Procedure): pass

class pick_green(Procedure): pass

class pick_blue(Procedure): pass

class go_bucket(Procedure): pass

class gen_block(Procedure): pass

class start_position(Procedure): pass

class reset_all(Procedure): pass

class target_reached(Reactor): pass

class bucket_reached(Reactor): pass

class nearest_target_reached(Reactor): pass

class nearest_red_picked(Reactor): pass

class nearest_green_picked(Reactor): pass

class nearest_blue_picked(Reactor): pass

class color(Reactor): pass

def_vars('F', 'X', 'C', 'Y', 'destinations', 'dest', 'redlist', 'greenlist', 'bluelist', 'green', 'blue', 'red', 'position_in_range')

class main(Agent):
    def main(self):
        
        #Procedura per generare i dischi colorati
        gen_block() >> [+generate_blocks()[{'to': 'robot@127.0.0.1:6566'}]]
        
        #Procedura per scansionare le 20 possibili posizioni
        scan() >> [
            "destinations=[]", 
            get_vertex(destinations), 
            reset(), 
            +scan_nearest_position(destinations)[{'to': 'robot@127.0.0.1:6566'}] 
        ]
        +nearest_target_reached(dest)[{'from':F}] >> [
            sense(dest)
        ]
        
        #Procedura per prendere i dischi e posizionarli nel bucket
        pick() >> [
            pick_red()
        ]
        
        #Procedura per riportare il robot alla posizione iniziale
        start_position() >> [+go_to_start()[{'to': 'robot@127.0.0.1:6566'}]]
        
        #Questa procedura consente di aggiungere alla lista destination tutte le posizioni non visitate dal robot
        get_vertex(destinations) / (block(X, C) & eq(C, "Not_Visited")) >> [
            "destinations.append(X)",
            -block(X,"Not_Visited"),
            +block(X,"Add_To_Destinations"),
            get_vertex(destinations)
        ]
        
        #Quando il robot si trova in prossimità del disco aggiorno la KB del robot con il colore rilevato
        +color(X,C)[{'from':F}] >> [
            "position_in_range = X - 31",
            show_line("Il Robot è andato alla posizione: ", position_in_range, " ed ha rilevato il colore: ", C),
            -block(X,'Not_Visited'),
            +block(X,C),
            scan()
        ]
        
        #Con questa procedura faccio un reset delle posizioni che sono state aggiunte tra le possibili destinazioni ma che in realtà non sono state visitate 
        reset() / (block(X, C) & eq(C, "Add_To_Destinations")) >> [
            -block(X,"Add_To_Destinations"),
            +block(X,"Not_Visited"),
            reset()
        ]
        
        #Procedura per poter raccogliere i dischi rossi
        pick_red() / (block(X, C) & eq(C, "red")) >> ["redlist=[]", get_red(redlist), reset_red(), +go_to_nearest_block(redlist)[{'to': 'robot@127.0.0.1:6566'}] ]
        +nearest_red_picked(red)[{'from':F}] >> [ 
            -block(red,'red'),
            +block(red,'picked'),
            show_line("Red Picked"),
            go_bucket(1),
            show_line("Red Released"),
            pick_red()
        ]
        
        pick_red() >> [pick_green()]
        
        #Procedura per poter raccogliere i dischi verdi
        pick_green() / (block(X, C) & eq(C, "green")) >> ["greenlist=[]", get_green(greenlist), reset_green(), +go_to_nearest_block(greenlist)[{'to': 'robot@127.0.0.1:6566'}] ]
        +nearest_green_picked(green)[{'from':F}] >> [ 
            -block(green,'green'),
            +block(green,'picked'),
            show_line("Green Picked"),
            go_bucket(2),
            show_line("Green Released"),
            pick_green()
        ]
        
        pick_green() >> [pick_blue()]
        
        #Procedura per poter raccogliere i dischi blu
        pick_blue() / (block(X, C) & eq(C, "blue")) >> ["bluelist=[]", get_blue(bluelist), reset_blue(), +go_to_nearest_block(bluelist)[{'to': 'robot@127.0.0.1:6566'}] ]
        +nearest_blue_picked(blue)[{'from':F}] >> [ 
            -block(blue,'blue'),
            +block(blue,'picked'),
            show_line("Blue Picked"),
            go_bucket(3),
            show_line("Blue Released"),
            pick_blue()
        ]
        
        pick_blue() >> [
            start_position(),
            show_line("Pick Operation Completed")
        ]
        
        #Procedura per poter raggiungere uno specifico bucket
        #1 = Rosso
        #2 = Verde
        #3 = Blue
        go_bucket(Y) >> [ +go_to_bucket(Y)[{'to': 'robot@127.0.0.1:6566'}] ]
        +bucket_reached() >> [{'from':F}] >> [ 
            show_line("Bucket Raggiunto")
        ]
        
        #Procedura per sfruttare il sensore di colore
        sense(dest) >> [+sense_color(dest)[{'to': 'robot@127.0.0.1:6566'}]]
        
        #Procedura per prendere la lista dei dischi rossi non ancora inseriti nel bucket
        get_red(redlist) / (block(X, C) & eq(C, "red")) >> [
            "redlist.append(X)",
            -block(X,"red"),
            +block(X,"red_candidate"),
            get_red(redlist)
        ]
        
        #Procedura per prendere la lista dei dischi verdi non ancora inseriti nel bucket
        get_green(greenlist) / (block(X, C) & eq(C, "green")) >> [
            "greenlist.append(X)",
            -block(X,"green"),
            +block(X,"green_candidate"),
            get_green(greenlist)
        ]
        
        #Procedura per prendere la lista dei dischi blu non ancora inseriti nel bucket
        get_blue(bluelist) / (block(X, C) & eq(C, "blue")) >> [
            "bluelist.append(X)",
            -block(X,"blue"),
            +block(X,"blue_candidate"),
            get_blue(bluelist)
        ]
        
        #Procedura per resettare la lista dei dischi rossi non ancora inseriti nel bucket
        reset_red() / (block(X, C) & eq(C, "red_candidate")) >> [
            -block(X,"red_candidate"),
            +block(X,"red"),
            reset_red()
        ]
        
        #Procedura per resettare la lista dei dischi verdi non ancora inseriti nel bucket
        reset_green() / (block(X, C) & eq(C, "green_candidate")) >> [
            -block(X,"green_candidate"),
            +block(X,"green"),
            reset_green()
        ]
        
        #Procedura per resettare la lista dei dischi blu non ancora inseriti nel bucket
        reset_blue() / (block(X, C) & eq(C, "blue_candidate")) >> [
            -block(X,"blue_candidate"),
            +block(X,"blue"),
            reset_blue()
        ]
        
        #Procedura per resettare tutto e ripartire da zero
        reset_all() / (block(X, C) & (lambda: (C == "picked" or C == "None"))) >> [
            -block(X,"picked"),
            -block(X,"None"),
            +block(X,"Not_Visited"),
            reset_all()
        ]
        
        reset_all() >> [show_line("Reset All Completed")]
        
ag = main()
ag.start()
# Un blocco è identificato dal vertice in cui è posizionato e da un colore
ag.assert_belief(block(32, "Not_Visited"))
ag.assert_belief(block(33, "Not_Visited"))
ag.assert_belief(block(34, "Not_Visited"))
ag.assert_belief(block(35, "Not_Visited"))
ag.assert_belief(block(36, "Not_Visited"))
ag.assert_belief(block(37, "Not_Visited"))
ag.assert_belief(block(38, "Not_Visited"))
ag.assert_belief(block(39, "Not_Visited"))
ag.assert_belief(block(40, "Not_Visited"))
ag.assert_belief(block(41, "Not_Visited"))
ag.assert_belief(block(42, "Not_Visited"))
ag.assert_belief(block(43, "Not_Visited"))
ag.assert_belief(block(44, "Not_Visited"))
ag.assert_belief(block(45, "Not_Visited"))
ag.assert_belief(block(46, "Not_Visited"))
ag.assert_belief(block(47, "Not_Visited"))
ag.assert_belief(block(48, "Not_Visited"))
ag.assert_belief(block(49, "Not_Visited"))
ag.assert_belief(block(50, "Not_Visited"))
ag.assert_belief(block(51, "Not_Visited"))
PHIDIAS.run_net(globals(), 'http')
PHIDIAS.shell(globals())