import os
import sys
from e2xostream.stk.scenariogeneration import xosc, prettyprint, ScenarioGenerator
from e2xostream.config.default_properties import DEFAULT_SIZE_PROPERTIES


class EntityFactory:
    def __init__(self):
        pass

    def create_entity(self, entity_type, name, properties, category=None, model=None, vehicle_file=None, default_cat=False, **kwargs):
        """
        Creates an entity of a given type with the specified properties.

        Parameters:
        - entity_type (str): Type of the entity (Vehicle, Pedestrian, MiscObject, ExternalObjectReference).
        - name (str): Name of the entity.
        - properties (dict): Properties to be added to the entity.
        - category (str): Category of the entity (e.g., vehicle category).
        - model (str): Model of the entity.
        - vehicle_file (str): Path to the vehicle model file.
        - kwargs: Additional parameters specific to the entity type.

        Returns:
        - The created entity object.
        """
        bb, fa, ba = None, None, None

        if default_cat:
            if category in DEFAULT_SIZE_PROPERTIES:
                params = DEFAULT_SIZE_PROPERTIES[category]
                width, length, height = params['width'], params['length'], params['height']
                bb = xosc.BoundingBox(width, length, height, 1.3, 0, 0.8)
                fa = xosc.Axle(0, 0, 0, 0, 0)
                ba = xosc.Axle(0, 0, 0, 0, 0)
        else:
            width, length, height = kwargs['width'], kwargs['length'], kwargs['height']
            bb = xosc.BoundingBox(width, length, height, 1.3, 0, 0.8)
            fa = xosc.Axle(0, 0, 0, 0, 0)
            ba = xosc.Axle(0, 0, 0, 0, 0)

        if entity_type == 'Vehicle':
            vehicle_category_attribute = getattr(xosc.VehicleCategory, category, None)
            if vehicle_category_attribute is not None:
                entity = xosc.Vehicle(name, vehicle_category_attribute, bb, fa, ba, kwargs.get('max_speed', 70), kwargs.get('max_acceleration', 10), kwargs.get('max_deceleration', 10))
            else:
                raise ValueError("Vehicle category not available")
        elif entity_type == 'Pedestrian':
            pedestrian_category_attribute = getattr(xosc.PedestrianCategory, category, None)
            if pedestrian_category_attribute is not None:
                entity = xosc.Pedestrian(name, kwargs.get('mass', 70), pedestrian_category_attribute, bb, model)
            else:
                raise ValueError("Pedestrian category not available")
        elif entity_type == 'MiscObject':
            misc_object_category_attribute = getattr(xosc.MiscObjectCategory, category, None)
            if misc_object_category_attribute is not None:
                entity = xosc.MiscObject(name, kwargs.get('mass', 70), misc_object_category_attribute, bb, model)
            else:
                raise ValueError("Misc object category not available")
        elif entity_type == 'ExternalObjectReference':
            if name:
                entity = xosc.ExternalObjectReference(name)
            else:
                raise ValueError("Name is required for ExternalObjectReference")
        else:
            raise ValueError("Unsupported entity type")

        for k, v in properties.items():
            entity.add_property(k, v)

        return entity


# Usage examples
if __name__ == "__main__":
    factory = EntityFactory()

    # Example 1: Creating a Vehicle Entity
    vehicle_properties = {'color': 'red', 'model': 'sedan'}
    vehicle = factory.create_entity(
        entity_type='Vehicle',
        name='RedCar',
        properties=vehicle_properties,
        category='car',
        model="ENCAP_GVT01",
        vehicle_file="../../resources/models/car_red.osgb",
        max_speed=70,
        max_acceleration=10,
        max_deceleration=10
    )
    print(vehicle)

    # Example 2: Creating a Pedestrian Entity
    pedestrian_properties = {'age': '30', 'gender': 'male'}
    pedestrian = factory.create_entity(
        entity_type='Pedestrian',
        name='JohnDoe',
        properties=pedestrian_properties,
        category='adult',
        model="PED_MODEL",
        mass=70
    )
    print(pedestrian)

    # Example 3: Creating a MiscObject Entity
    misc_object_properties = {'material': 'metal', 'purpose': 'barrier'}
    misc_object = factory.create_entity(
        entity_type='MiscObject',
        name='Barrier',
        properties=misc_object_properties,
        category='barrier',
        model="BARRIER_MODEL",
        mass=100
    )
    print(misc_object)

    # Example 4: Creating an ExternalObjectReference Entity
    external_object_properties = {'source': 'external'}
    external_object = factory.create_entity(
        entity_type='ExternalObjectReference',
        name='ExternalObject01',
        properties=external_object_properties
    )
    print(external_object)

