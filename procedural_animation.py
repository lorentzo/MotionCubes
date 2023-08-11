
import bpy
import mathutils
import bmesh

# Inspiration: https://www.youtube.com/@YFJSR/videos

# TODO:
# * create cubes into collections
# * animate creation of new cubes
# * Add blinking emission when new cubes are created
# * add colors to cubes
# * add small bouncing when new cubes are created
# * experiment with local translations for created cubes

def delete_objects(objects):
    for obj in objects:
        bpy.data.objects.remove(bpy.data.objects[obj.name], do_unlink=True)

def create_cube(size=2, location=mathutils.Vector((0,0,0))):
    # Make a new BMesh.
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=size, matrix=mathutils.Matrix.Identity(4), calc_uvs=False)
    # Transform.
    bmesh.ops.translate(
        bm,
        verts=bm.verts,
        vec=location)
    # Write the bmesh into a new mesh.
    me = bpy.data.meshes.new("Mesh")
    bm.to_mesh(me)
    bm.free()
    # Add the mesh to the scene.
    obj = bpy.data.objects.new("Object", me)
    bpy.context.collection.objects.link(obj)
    return obj

def get_objects_from_collection(collection_name):
    collection_objects = []
    print("From collection", collection_name, "taking:")
    for obj in bpy.data.collections[collection_name].all_objects:
        print(obj.name)
        collection_objects.append(obj)
    return collection_objects

def create_8_cubes_from_cube(starting_cube):
    """
        Given cube, create 8 smaller cubes which fit inside the given cube.
        NOTE: not working!
    """
    # Figure out edge length of a cube.
    edge_vertices = starting_cube.data.edges[0].vertices
    v1 = starting_cube.data.vertices[edge_vertices[0]]
    v2 = starting_cube.data.vertices[edge_vertices[1]]
    edge_length = mathutils.Vector(v1.co - v2.co).length
    # Figure out the center of the cube.
    cube_center_world = mathutils.Vector((0,0,0))
    for v in starting_cube.data.vertices:
        cube_center_world += starting_cube.matrix_world @ v.co 
    cube_center_world /= 8.0
    #bpy.ops.mesh.primitive_uv_sphere_add(radius=0.3, enter_editmode=False, align='WORLD', location=cube_center_world, scale=(1, 1, 1))

    # Create 8 smaller cubes.
    new_cubes = []
    for v in starting_cube.data.vertices:
        v_co_world = starting_cube.matrix_world @ v.co
        #bpy.ops.mesh.primitive_uv_sphere_add(radius=0.3, enter_editmode=False, align='WORLD', location=v_co_world, scale=(1, 1, 1))
        new_cube_center_shift_from_vert_vec = mathutils.Vector(cube_center_world - v_co_world)
        new_cube_center_shift_from_vert_len = new_cube_center_shift_from_vert_vec.length
        new_cube_center_shift_from_vert_vec_norm = new_cube_center_shift_from_vert_vec.normalized()
        new_cube_loc = v_co_world + new_cube_center_shift_from_vert_vec_norm * new_cube_center_shift_from_vert_len / 2.0
        #bpy.ops.mesh.primitive_uv_sphere_add(radius=0.3, enter_editmode=False, align='WORLD', location=new_cube_loc, scale=(1, 1, 1))
        new_cube = create_cube(size=edge_length/2.0, location=new_cube_loc)
        new_cubes.append(new_cube)

    return new_cubes

def create_8_cubes_from_cube2(starting_cube):

    # Figure out edge length of a cube.
    edge_vertices = starting_cube.data.edges[0].vertices
    v1 = starting_cube.data.vertices[edge_vertices[0]]
    v1_co_world = starting_cube.matrix_basis @ v1.co 
    v2 = starting_cube.data.vertices[edge_vertices[1]]
    v2_co_world = starting_cube.matrix_basis @ v2.co 
    edge_length = mathutils.Vector(v1_co_world - v2_co_world).length    

    # Create smaller cubes.
    local_translation_magnitude = edge_length/4.0
    local_translations = [
        mathutils.Vector((-local_translation_magnitude, -local_translation_magnitude, -local_translation_magnitude)),
        mathutils.Vector((+local_translation_magnitude, +local_translation_magnitude, +local_translation_magnitude)),
        mathutils.Vector((+local_translation_magnitude, -local_translation_magnitude, -local_translation_magnitude)),
        mathutils.Vector((+local_translation_magnitude, +local_translation_magnitude, -local_translation_magnitude)),
        mathutils.Vector((-local_translation_magnitude, +local_translation_magnitude, -local_translation_magnitude)),
        mathutils.Vector((-local_translation_magnitude, -local_translation_magnitude, +local_translation_magnitude)),
        mathutils.Vector((+local_translation_magnitude, -local_translation_magnitude, +local_translation_magnitude)),
        mathutils.Vector((-local_translation_magnitude, +local_translation_magnitude, +local_translation_magnitude))
    ]
    world_translation = starting_cube.matrix_basis.to_translation()
    world_rotation = starting_cube.matrix_basis.to_euler()
    print(starting_cube.matrix_basis)
    new_cubes = []
    for i in range(8):
        new_cube = create_cube(size=edge_length/2.0, location=mathutils.Vector((0,0,0)))
        new_cube.location += world_translation
        new_cube.rotation_euler = world_rotation
        local_translations[i].rotate(starting_cube.matrix_basis) # ensure that translation is local
        new_cube.location += local_translations[i]
        new_cubes.append(new_cube)
    
    return new_cubes
        

def main():
    starting_cubes = get_objects_from_collection("starting_cubes")

    # Create rigid body world.
    bpy.ops.rigidbody.world_remove()
    bpy.ops.rigidbody.world_add()
    bpy.context.scene.rigidbody_world.enabled = True
    collection = bpy.data.collections.new("RigidBodyCubeCollection")
    bpy.context.scene.rigidbody_world.collection = collection

    working_cubes = []
    storage_cubes = []

    # Add starting cubes to rigid body physics.
    for cube_i in starting_cubes:
        bpy.context.scene.rigidbody_world.collection.objects.link(cube_i)
        working_cubes.append(cube_i)

    n_steps = 2
    curr_step = 0
    while True:
        for working_cube in working_cubes:
            new_cubes = create_8_cubes_from_cube2(working_cube)
            for new_cube in new_cubes:
                bpy.context.scene.rigidbody_world.collection.objects.link(new_cube)
                storage_cubes.append(new_cube)
        delete_objects(working_cubes)
        working_cubes.clear()
        working_cubes.extend(storage_cubes)
        storage_cubes.clear()
        curr_step += 1
        if curr_step > n_steps:
            break




if __name__ == "__main__":
    main()
