import bpy
import math
import random
import sys
import csv

for i in range(500):


    bpy.ops.wm.read_factory_settings(use_empty=True)

    scene = bpy.context.scene
    scene.objects.keys()

    ter_file_loc = ""
    if i < 10:
        ter_file_loc = "OUTPUT/" + 'ter-000' + str(i)+'.obj'
    elif i < 100:
        ter_file_loc = "OUTPUT/" + 'ter-00' + str(i)+'.obj'
    else:
        ter_file_loc = "OUTPUT/" + 'ter-0' + str(i)+'.obj'

    imported_object = bpy.ops.import_scene.obj(filepath=ter_file_loc)
    obj_object = bpy.context.object
    #bpy.ops.transform.rotate(value=(math.pi * 0.5), orient_axis='X')  # value = Angle

    wheel_csv_file_loc = ""
    wheel_obj_file_loc = "cyl-wheel.obj"

    if i < 10:
        wheel_csv_file_loc = "OUTPUT/" + "whe-000"+str(i)+".csv"
    elif i < 100:
        wheel_csv_file_loc = "OUTPUT/" + "whe-00"+str(i)+".csv"
    else:
        wheel_csv_file_loc = "OUTPUT/" + "whe-0"+str(i)+".csv"


    output = []
    with open(wheel_csv_file_loc, 'r') as pos_file:
        reader = csv.reader(pos_file)
        for row in reader:
            output=row

    pos_x = float(output[0])
    pos_y = float(output[1])
    pos_z = float(output[2])


    imported_object_0 = bpy.ops.import_scene.obj(filepath=wheel_obj_file_loc)
    bpy.ops.transform.translate(value=(pos_x,-pos_z , pos_y))
    #bpy.ops.transform.rotate(value=(-math.pi * 0.5), orient_axis='X')  # value = Angle

    #bpy.ops.mesh.primitive_plane_add(size=200.0, calc_uvs=True, enter_editmode=False, align='WORLD',
    #                                     location=(0.0, 0.0, -2.0))


    bpy.ops.object.camera_add(enter_editmode=False, align='WORLD', location=(1.9202, -1.80555 , -4.24172),
                                rotation=(-3.3684854563, 0, 6.283186), scale=(1.0, 1.0, 1.0))
    scene.camera = bpy.context.object
    scene.cycles.device = 'GPU'

    prefs = bpy.context.preferences
    cprefs = prefs.addons['cycles'].preferences

    # Attempt to set GPU device types if available
    for compute_device_type in ('CUDA', 'OPENCL', 'NONE'):
        try:
            cprefs.compute_device_type = compute_device_type
            break
        except TypeError:
            pass

    # Enable all CPU and GPU devices
    cprefs.get_devices()
    for device in cprefs.devices:
        device.use = True

    # create light datablock, set attributes
    light_data = bpy.data.lights.new(name="light_2.80", type='POINT')
    light_data.energy = 300

    # create new object with our light datablock
    light_object = bpy.data.objects.new(name="light_2.80", object_data=light_data)

    # link light object
    bpy.context.collection.objects.link(light_object)

    # make it active
    bpy.context.view_layer.objects.active = light_object

    # change location
    light_object.location = (-0.182606 , -2.1494 , 0.549828)

    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.cycles.device = 'GPU'
    # bpy.context.scene.render.resolution_percentage = 200
    bpy.context.scene.cycles.samples = 128
    bpy.context.scene.render.resolution_x = 1280
    bpy.context.scene.render.resolution_y = 720
    bpy.context.scene.render.filepath = "image_file/"+str(i) +".png"
    # bpy.context.scene.render.image_settings.compression = 50
    bpy.context.scene.render.image_settings.color_mode = 'RGBA'
    bpy.context.scene.render.image_settings.file_format = 'PNG'
    bpy.ops.render.render(write_still=True)
