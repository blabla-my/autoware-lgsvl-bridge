"""
The Config module (for specifying the fuzzing configurations)
"""
# Tool Description
__description__ = 'Simulation-based fuzzing for autoware'

# Tool Version
__version__ = '1.0.0'

# Tool Name
__prog__ = 'autoware-runner'

# Mutate Probability
PROB_PedestrianVariant = 30
PROB_PedestrianPisition = 30
PROB_PedestrianSpeed = 50

PROB_TrafficConePosition = 30

PROB_WeatherRain = 30
PROB_WeatherFog = 30
PROB_WeatherWetness = 30
PROB_WeatherCloudiness = 30

PROB_TimeMonth = 30
PROB_TimeDay = 30
PROB_TimeHour = 30
PROB_TimeMinute = 30
PROB_TimeSecond = 30

PROB_NPCVariant = 30
PROB_NPCColor = 30
PROB_NPCPosition = 30
PROB_NPCSpeed = 50
PROB_NPCAdd = 10
PROB_NPCDelete = 10
PROB_NPCMaxSpeed = 30

# FIXME cones are not percepted
PROB_TrafficLightState = 30
PROB_TrafficLightValue = 30
PROB_TrafficConeAdd = 0
PROB_TrafficConeDelete = 30

MAX_OBSTACLE_NUM = 10
MAX_NPC_NUM = 10

