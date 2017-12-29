import executive_engine_api as api
import numpy as np

def runMission():  

  api.activateBehavior('SELF_LOCALIZE_BY_ODOMETRY')
  api.executeBehavior('TAKE_OFF')

  targetPosition1=[3, 2, 1]  
  targetPosition2=[1, 5, 1]
  targetPosition3=[4, 9, 1]

  api.executeBehavior('GO_TO_POINT', coordinates=targetPosition1)
  #zczytaj pozycje po raz pierwszy
  success, unification = api.consultBelief('position(self, (?X, ?Y, ?Z))') #sprawdz znow  
  if success:
    a=unification['X']
    b=unification['Y']
    c=unification['Z']         
    g=[a, b, c] #zapisz do wektora
    np.savetxt('/home/jacek/Pulpit/dokument1.txt', np.array(g), fmt='%.2f') #musi byc pelna sciezka
  
  
  api.executeBehavior('GO_TO_POINT', coordinates=targetPosition2)
  
  success, unification = api.consultBelief('position(self, (?X, ?Y, ?Z))') #sprawdz znow  
  if success:
    a=unification['X']
    b=unification['Y']
    c=unification['Z']         
    g=[a, b, c] #zapisz do wektora
    np.savetxt('/home/jacek/Pulpit/dokument1.txt', np.array(g), fmt='%.2f') #musi byc pelna sciezka

  
  
  api.executeBehavior('GO_TO_POINT', coordinates=targetPosition3)
  
  success, unification = api.consultBelief('position(self, (?X, ?Y, ?Z))') #sprawdz znow  
  if success:
    a=unification['X']
    b=unification['Y']
    c=unification['Z']         
    g=[a, b, c] #zapisz do wektora
    np.savetxt('/home/jacek/Pulpit/dokument1.txt', np.array(g), fmt='%.2f') #musi byc pelna sciezka
  
  
  api.executeBehavior('LAND')
  
  api.inhibitBehavior('SELF_LOCALIZE_BY_ODOMETRY')

  

