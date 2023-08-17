 : : : INSTANCES INVOLVED IN THE NEEM : : 

kb_project(instance_of(map_bringing_object:'StretchRobot', dul:'PhysicalAgent')). 

kb_project(instance_of(map_bringing_object:'Location_of_StretchRobot', dul:'Place')).


kb_project(instance_of(map_bringing_object:'soft_drink_cola', ocra_home:'Drink')).
kb_project(instance_of(map_bringing_object:'matcha_tea', ocra_home:'Drink')). 

kb_project(instance_of(map_bringing_object:'Flavor_of_soft_drink_cola', dul:'Quality')).
kb_project(instance_of(map_bringing_object:'Temperature_of_soft_drink_cola', dul:'Quality')).
kb_project(instance_of(map_bringing_object:'Location_of_soft_drink_cola', dul:'Place')).

kb_project(instance_of(map_bringing_object:'Flavor_of_matcha_tea', dul:'Quality')).
kb_project(instance_of(map_bringing_object:'Temperature_of_matcha_tea', dul:'Quality')).
kb_project(instance_of(map_bringing_object:'Location_of_matcha_tea', dul:'Place')).

kb_project(instance_of(map_bringing_object:'UltraProcessedDrink', dul:'Role')).
kb_project(instance_of(map_bringing_object:'TastyDrink', dul:'Role')).
kb_project(instance_of(map_bringing_object:'ProcessedDrink', dul:'Role')).
kb_project(instance_of(map_bringing_object:'HealthyDrink', dul:'Role')).


: : : : StretchRobot : : 

kb_project(triple(map_bringing_object:'StretchRobot', dul:'hasLocation', map_bringing_object:'Location_of_StretchRobot')).


: : : : SoftDrink-Cola : : 

kb_project(triple(map_bringing_object:'soft_drink_cola', ocra_home:'hasFlavor', map_bringing_object:'Flavor_of_soft_drink_cola')).
kb_project(triple(map_bringing_object:'soft_drink_cola', ocra_home:'hasTemperature', map_bringing_object:'Temperature_of_soft_drink_cola')).
kb_project(triple(map_bringing_object:'soft_drink_cola', dul:'hasLocation', map_bringing_object:'Location_of_soft_drink_cola')).
kb_project(triple(map_bringing_object:'soft_drink_cola', dul:'hasRole', map_bringing_object:'UltraProcessedDrink')).
kb_project(triple(map_bringing_object:'soft_drink_cola', dul:'hasRole', map_bringing_object:'TastyDrink')).

kb_project(triple(map_bringing_object:'Flavor_of_soft_drink_cola', ocra_home:'hasDataValue', map_bringing_object:'sweet')).
kb_project(triple(map_bringing_object:'Temperature_of_soft_drink_cola', ocra_home:'hasDataValue', map_bringing_object:'cold')).


: : : : MatchaTea : : 

kb_project(triple(map_bringing_object:'matcha_tea', ocra_home:'hasFlavor', map_bringing_object:'Flavor_of_matcha_tea')).
kb_project(triple(map_bringing_object:'matcha_tea', ocra_home:'hasTemperature', map_bringing_object:'Temperature_of_matcha_tea')).
kb_project(triple(map_bringing_object:'matcha_tea', dul:'hasLocation', map_bringing_object:'Location_of_matcha_tea')).
kb_project(triple(map_bringing_object:'matcha_tea', dul:'hasRole', map_bringing_object:'ProcessedDrink')).
kb_project(triple(map_bringing_object:'matcha_tea', dul:'hasRole', map_bringing_object:'HealthyDrink')).

kb_project(triple(map_bringing_object:'Flavor_of_matcha_tea', ocra_home:'hasDataValue', map_bringing_object:'bitter')).
kb_project(triple(map_bringing_object:'Flavor_of_matcha_tea', ocra_home:'hasDataValue', map_bringing_object:'savory')).
kb_project(triple(map_bringing_object:'Temperature_of_matcha_tea', ocra_home:'hasDataValue', map_bringing_object:'hot')).


: : : : C : : 

kb_project(triple(map_bringing_object:'Location_of_soft_drink_cola', dul:'farFrom', map_bringing_object:'Location_of_StretchRobot')).
kb_project(triple(map_bringing_object:'Location_of_matcha_tea', dul:'nearTo', map_bringing_object:'Location_of_StretchRobot')).

kb_project(triple(map_bringing_object:'soft_drink_cola', ocra_home:'isMoreProcessedThan', map_bringing_object:'matcha_tea')).
kb_project(triple(map_bringing_object:'soft_drink_cola', ocra_home:'isTastierThan', map_bringing_object:'matcha_tea')).
kb_project(triple(map_bringing_object:'soft_drink_cola', ocra_home:'isColderThan', map_bringing_object:'matcha_tea')).

kb_project(triple(map_bringing_object:'matcha_tea', ocra_home:'isMoreNaturalThan', map_bringing_object:'soft_drink_cola')).
kb_project(triple(map_bringing_object:'matcha_tea', ocra_home:'isLessTastyThan', map_bringing_object:'soft_drink_cola')).
kb_project(triple(map_bringing_object:'matcha_tea', ocra_home:'isHotterThan', map_bringing_object:'soft_drink_cola')).