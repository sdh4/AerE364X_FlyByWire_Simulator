import sys
import copy
import os
import os.path
import threading

from matplotlib import pyplot as pl

import dynamic_model

def microcontroller_code():
    ucglobals = {}


    # !!!*** NON-REENTRANT
    # Temporarily adjust sys.path so as to add script's directory

    ourdir = os.path.split(__file__)[0]
    
    syspath_save=sys.path
    syspath_new=copy.deepcopy(syspath_save)
    syspath_new.insert(0,ourdir)
    sys.path=syspath_new

    mainfile = os.path.join(ourdir,"main.py")
    
    with open(mainfile) as f:
        code = compile(f.read(),mainfile,'exec')
        exec(code,ucglobals)
        pass
    

    sys.path = syspath_save

    sys.exit(0)
    pass

#print("main_thread=%d" % (id(threading.current_thread())))

sys.setswitchinterval(.0004) # improve context switch behavior
dynamic_model.dynamic_instance = dynamic_model.dynamic_model()

microcontroller_thread = threading.Thread(target=microcontroller_code)
microcontroller_thread.start()

dynamic_model.dynamic_instance.interactor.interactor() # calls pl.show()

