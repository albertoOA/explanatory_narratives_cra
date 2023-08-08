 : : : INSTANCES INVOLVED IN THE NEEM : : 

kb_project(instance_of(map_bringing_object:'StretchRobot', dul:'PhysicalAgent')). 

kb_project(instance_of(map_bringing_object:'Location_of_StretchRobot', dul:'Place')).


kb_project(instance_of(map_bringing_object:'SoftDrink_Cola', ocra_home:'Drink')).
kb_project(instance_of(map_bringing_object:'MatchaTea', ocra_home:'Drink')). 

kb_project(instance_of(map_bringing_object:'Flavor_of_SoftDrink_Cola', dul:'Quality')).
kb_project(instance_of(map_bringing_object:'Temperature_of_SoftDrink_Cola', dul:'Quality')).
kb_project(instance_of(map_bringing_object:'Location_of_SoftDrink_Cola', dul:'Place')).

kb_project(instance_of(map_bringing_object:'Flavor_of_MatchaTea', dul:'Quality')).
kb_project(instance_of(map_bringing_object:'Temperature_of_MatchaTea', dul:'Quality')).
kb_project(instance_of(map_bringing_object:'Location_of_MatchaTea', dul:'Place')).


: : : : StretchRobot : : 

kb_project(triple(map_bringing_object:'StretchRobot', dul:'hasLocation', map_bringing_object:'Location_of_StretchRobot')).


: : : : SoftDrink-Cola : : 

kb_project(triple(map_bringing_object:'SoftDrink_Cola', ocra_home:'hasFlavor', map_bringing_object:'Flavor_of_SoftDrink_Cola')).
kb_project(triple(map_bringing_object:'SoftDrink_Cola', ocra_home:'hasTemperature', map_bringing_object:'Temperature_of_SoftDrink_Cola')).
kb_project(triple(map_bringing_object:'SoftDrink_Cola', dul:'hasLocation', map_bringing_object:'Location_of_SoftDrink_Cola')).
kb_project(triple(map_bringing_object:'SoftDrink_Cola', dul:'hasRole', map_bringing_object:'UltraProcessedDrink')).
kb_project(triple(map_bringing_object:'SoftDrink_Cola', dul:'hasRole', map_bringing_object:'TastyDrink')).

kb_project(triple(map_bringing_object:'Flavor_of_SoftDrink_Cola', ocra_home:'hasDataValue', map_bringing_object:'sweet')).
kb_project(triple(map_bringing_object:'Temperature_of_SoftDrink_Cola', ocra_home:'hasDataValue', map_bringing_object:'cold')).


: : : : MatchaTea : : 

kb_project(triple(map_bringing_object:'MatchaTea', ocra_home:'hasFlavor', map_bringing_object:'Flavor_of_MatchaTea')).
kb_project(triple(map_bringing_object:'MatchaTea', ocra_home:'hasTemperature', map_bringing_object:'Temperature_of_MatchaTea')).
kb_project(triple(map_bringing_object:'MatchaTea', dul:'hasLocation', map_bringing_object:'Location_of_MatchaTea')).
kb_project(triple(map_bringing_object:'MatchaTea', dul:'hasRole', map_bringing_object:'ProcessedDrink')).
kb_project(triple(map_bringing_object:'MatchaTea', dul:'hasRole', map_bringing_object:'HealthyDrink')).

kb_project(triple(map_bringing_object:'Flavor_of_MatchaTea', ocra_home:'hasDataValue', map_bringing_object:'bitter')).
kb_project(triple(map_bringing_object:'Flavor_of_MatchaTea', ocra_home:'hasDataValue', map_bringing_object:'savory')).
kb_project(triple(map_bringing_object:'Temperature_of_MatchaTea', ocra_home:'hasDataValue', map_bringing_object:'hot')).


: : : : C : : 

kb_project(triple(map_bringing_object:'Location_of_SoftDrink_Cola', dul:'farFrom', map_bringing_object:'Location_of_StretchRobot')).
kb_project(triple(map_bringing_object:'Location_of_MatchaTea', dul:'nearTo', map_bringing_object:'Location_of_StretchRobot')).

kb_project(triple(map_bringing_object:'SoftDrink_Cola', ocra_home:'isMoreProcessedThan', map_bringing_object:'MatchaTea')).
kb_project(triple(map_bringing_object:'SoftDrink_Cola', ocra_home:'isTastierThan', map_bringing_object:'MatchaTea')).
kb_project(triple(map_bringing_object:'SoftDrink_Cola', ocra_home:'isColderThan', map_bringing_object:'MatchaTea')).

kb_project(triple(map_bringing_object:'MatchaTea', ocra_home:'isMoreNaturalThan', map_bringing_object:'SoftDrink_Cola')).
kb_project(triple(map_bringing_object:'MatchaTea', ocra_home:'isLessTastyThan', map_bringing_object:'SoftDrink_Cola')).
kb_project(triple(map_bringing_object:'MatchaTea', ocra_home:'isHotterThan', map_bringing_object:'SoftDrink_Cola')).