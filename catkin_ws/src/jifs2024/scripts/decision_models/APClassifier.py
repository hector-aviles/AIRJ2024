from sklearn.base import BaseEstimator, ClassifierMixin

class RuleClassifier(BaseEstimator, ClassifierMixin):
    def __init__(self):
        # Define your rules here
        pass
    
    def fit(self, X, y=None):
        # This method is required by scikit-learn, but we don't need to do anything in this case
        return self
    
    def predict(self, X):
        # Implement your prediction logic here
        predictions = []
               
        for row in X.iterrows():
            # Example of a simple rule: if Feature1 > 0 and Feature2 < 10, predict class 1, otherwise predict class 0
            curr_lane = row[1]['curr_lane']
            free_NW = row[1]['free_NW']
            free_W = row[1]['free_W']          
            free_SW = row[1]['free_SW']
            free_E = row[1]['free_E']
            free_NE = row[1]['free_NE']
            free_SE = row[1]['free_SE']                

            # left lane
            if not curr_lane:
           
              if not free_E and not free_NE and not free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and not free_NE and free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and not free_NE and free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and not free_NE and free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and not free_NE and free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and not free_NE and free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and not free_NE and free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and not free_NE and free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and not free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and free_NE and not free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and free_NE and not free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and free_NE and not free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and free_NE and not free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and free_NE and not free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and free_NE and not free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and free_NE and not free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and free_NE and free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and not free_NE and not free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and not free_NE and free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and not free_NE and free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and not free_NE and free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and not free_NE and free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and not free_NE and free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and not free_NE and free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and not free_NE and free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and not free_NW and not free_SE and not free_SW and not free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and not free_NW and not free_SE and not free_SW and free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and not free_NW and not free_SE and free_SW and not free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and not free_NW and not free_SE and free_SW and free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and not free_NW and free_SE and not free_SW and not free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and not free_NW and free_SE and not free_SW and free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and not free_NW and free_SE and free_SW and not free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and not free_NW and free_SE and free_SW and free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and free_NW and not free_SE and not free_SW and not free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and free_NW and not free_SE and not free_SW and free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and free_NW and not free_SE and free_SW and not free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and free_NW and not free_SE and free_SW and free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and free_NW and free_SE and not free_SW and not free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and free_NW and free_SE and not free_SW and free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and free_NW and free_SE and free_SW and not free_W:
                 predictions.append("Change_to_right")

              elif free_E and free_NE and free_NW and free_SE and free_SW and free_W:
                 predictions.append("Change_to_right")

                 
            # right lane      
            elif curr_lane:

              if not free_E and not free_NE and not free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and not free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and free_NW and not free_SE and not free_SW and free_W:
                 predictions.append("Change_to_left")

              elif not free_E and not free_NE and free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and free_NW and not free_SE and free_SW and free_W:
                 predictions.append("Change_to_left")

              elif not free_E and not free_NE and free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and free_NW and free_SE and not free_SW and free_W:
                 predictions.append("Change_to_left")

              elif not free_E and not free_NE and free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif not free_E and not free_NE and free_NW and free_SE and free_SW and free_W:
                 predictions.append("Change_to_left")

              elif not free_E and free_NE and not free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and not free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and not free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and not free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and not free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and not free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and not free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and not free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif not free_E and free_NE and free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and not free_NE and not free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and not free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and free_NW and not free_SE and not free_SW and free_W:
                 predictions.append("Change_to_left")

              elif free_E and not free_NE and free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and free_NW and not free_SE and free_SW and free_W:
                 predictions.append("Change_to_left")

              elif free_E and not free_NE and free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and free_NW and free_SE and not free_SW and free_W:
                 predictions.append("Change_to_left")

              elif free_E and not free_NE and free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Keep")

              elif free_E and not free_NE and free_NW and free_SE and free_SW and free_W:
                 predictions.append("Change_to_left")

              elif free_E and free_NE and not free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and not free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and not free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and not free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and not free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and not free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and not free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and not free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and free_NW and not free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and free_NW and not free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and free_NW and not free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and free_NW and not free_SE and free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and free_NW and free_SE and not free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and free_NW and free_SE and not free_SW and free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and free_NW and free_SE and free_SW and not free_W: 
                 predictions.append("Cruise")

              elif free_E and free_NE and free_NW and free_SE and free_SW and free_W: 
                 predictions.append("Cruise")



        return predictions        
        

