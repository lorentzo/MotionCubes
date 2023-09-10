
import bpy
import mathutils
import bmesh
import math

from numpy.random import default_rng

# Inspiration: https://www.youtube.com/@YFJSR/videos

# TODO:
# * animate creation of new cubes
# * Add blinking emission when new cubes are created
# * add colors to cubes
# * add small bouncing when new cubes are created
# * experiment with local translations for created cubes
# Experiment with collisions
# Experiment with rigid body as constraint to movement
# aDD SOUND when collide

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

# https://stackoverflow.com/questions/19045971/random-rounding-to-integer-in-python
def probabilistic_round(x):
    return int(math.floor(x + mathutils.noise.random()))

# https://github.com/blender/blender/blob/master/source/blender/nodes/geometry/nodes/node_geo_distribute_points_on_faces.cc
# base_obj - MUST BE TRIANGULATED!
# returns: list of touples: (p, N)
def mesh_uniform_sampling(base_obj, n_samples=10, base_density=5.0):
    rng = default_rng()
    samples = [] # (p, N)
    samples_all = []
    n_polygons = len(base_obj.data.polygons)
    samples_density = math.ceil(n_samples / n_polygons) + base_density
    for polygon in base_obj.data.polygons: # must be triangulated mesh!
        # Extract triangle vertices and their weights.
        triangle_vertices = []
        triangle_vertex_weights = []
        for v_idx in polygon.vertices:
            v = base_obj.data.vertices[v_idx]
            triangle_vertices.append(v.co)
            if len(v.groups) < 1:
                triangle_vertex_weights.append(0.0)
            else:
                triangle_vertex_weights.append(v.groups[0].weight) # TODO: only one group? Investigate! float in [0, 1], default 0.0
        # Create samples.
        point_amount = probabilistic_round(polygon.area + samples_density)
        for i in range(point_amount):
            a = mathutils.noise.random()
            b = mathutils.noise.random()
            c = mathutils.noise.random()
            s = a + b + c
            un = (a / s)
            vn = (b / s)
            wn = (c / s)
            p = un * triangle_vertices[0] + vn * triangle_vertices[1] + wn * triangle_vertices[2]
            w = un * triangle_vertex_weights[0] + vn * triangle_vertex_weights[1] + wn * triangle_vertex_weights[2] # interpolate weight
            n = polygon.normal # TODO: vertex normals?
            samples_all.append([p,n,w])
    print("Number of all samples:", len(samples_all), ". Number of desired samples:", n_samples)
    if len(samples_all) > n_samples:
        random_sample_indices = rng.integers(len(samples_all), size=n_samples)
        for i in random_sample_indices:
            samples.append(samples_all[i])
    else:
        print("Number of all samples is smaller than desired number of samples! Using only number of found samples")
        samples = samples_all
    return samples
        
def morph_to(particles, target_object):

    n_particles = len(particles)

    # Sample points on target object.
    samples = mesh_uniform_sampling(target_object, n_samples=n_particles, base_density=5.0)

    # Move particles to sampled points.
    for i in range(n_particles):
        particles[i].location = samples[i][0]
        particles[i].keyframe_insert("location", frame=100)

def create_material(mat_id, mat_type, color=mathutils.Color((1.0, 0.5, 0.1))):

    mat = bpy.data.materials.get(mat_id)

    if mat is None:
        mat = bpy.data.materials.new(name=mat_id)

    mat.use_nodes = True

    if mat.node_tree:
        mat.node_tree.links.clear()
        mat.node_tree.nodes.clear()

    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    output = nodes.new(type='ShaderNodeOutputMaterial')

    if mat_type == "diffuse":
        shader = nodes.new(type='ShaderNodeBsdfDiffuse')
        nodes["Diffuse BSDF"].inputs[0].default_value = color[:] + (1.0,)

    elif mat_type == "emission":
        shader = nodes.new(type='ShaderNodeEmission')
        nodes["Emission"].inputs[0].default_value = color[:] + (1.0,)
        nodes["Emission"].inputs[1].default_value = 5

    elif mat_type == "glossy":
        shader = nodes.new(type='ShaderNodeBsdfGlossy')
        nodes["Glossy BSDF"].inputs[0].default_value = color[:] + (1.0,)
        nodes["Glossy BSDF"].inputs[1].default_value = 0.2

    links.new(shader.outputs[0], output.inputs[0])

    return mat


def main():
    starting_cubes = get_objects_from_collection("starting_cubes")

    # Create rigid body world.
    if bpy.context.scene.rigidbody_world == None:
        bpy.ops.rigidbody.world_add()
        bpy.context.scene.rigidbody_world.enabled = True
        collection = bpy.data.collections.new("RigidBodyCubeCollection")
        bpy.context.scene.rigidbody_world.collection = collection
    bpy.context.scene.use_gravity = False

    working_cubes = []
    storage_cubes = []

    # Add starting cubes to rigid body physics.
    for cube_i in starting_cubes:
        bpy.context.scene.rigidbody_world.collection.objects.link(cube_i)
        working_cubes.append(cube_i)

    # Idea is to find hit point and then to further create more cubes on contact.
    # PROBLEM: It seems that rigid body can not be used this was.
    # Maybe using frame by frame rigid body or using kinematic body?
    """
    while True: 
        # https://blender.stackexchange.com/a/269349 - collision detection
        for working_cube in working_cubes:
            object_location, hitpoint, normal, has_hit = bpy.context.scene.rigidbody_world.convex_sweep_test(working_cube, (-20, -20, -20), (20, 20, 20))
            if has_hit:
                print(hitpoint)
    """

    # Create smaller cubes which become working cubes.
    n_steps = 3
    for i in range(n_steps):
        for working_cube in working_cubes:
            new_cubes = create_8_cubes_from_cube2(working_cube)
            for new_cube in new_cubes:
                bpy.context.scene.rigidbody_world.collection.objects.link(new_cube)
                storage_cubes.append(new_cube)
        delete_objects(working_cubes)
        working_cubes.clear()
        working_cubes.extend(storage_cubes)
        storage_cubes.clear()

    # Initialize working cubes material.
    for working_cube in working_cubes:
        col = mathutils.Color((1,1,1,))
        mat = None
        if mathutils.noise.random() > 0.0:
            mat = create_material(working_cube.name+"_mat", "glossy", color=col)
        else:
            mat = create_material(working_cube.name+"_mat", "emission", color=col)
        working_cube.data.materials.append(mat)

    # Initialize working cubes animation.
    for working_cube in working_cubes:
        working_cube.keyframe_insert("scale", frame=0)
        working_cube.keyframe_insert("rotation_euler", frame=0)
        working_cube.keyframe_insert("location", frame=0)

    # Initialize working cube physics.
    for working_cube in working_cubes:
        #working_cube.rigid_body.type = 'PASSIVE'
        working_cube.rigid_body.enabled = True
        #working_cube.rigid_body.kinematic = True
        #working_cube.rigid_body.collision_shape = 'MESH'
        working_cube.rigid_body.mass = 3000.0
        working_cube.rigid_body.friction = 0.3
        working_cube.rigid_body.restitution = 0.1
        working_cube.rigid_body.linear_damping = 1.0
        working_cube.rigid_body.angular_damping = 1.0        

    # Animation.
    # Idea1: morphing into another shape with rigid body as constraint.
    # Cubes are morphing at random times. When it is time to move this object is set to animated but interacting with other.
    # Problem: problematic rigid body in Blender.
    """
    morph_target = bpy.data.collections["morphing_target"].all_objects[0]
    morph_to(working_cubes, morph_target)
    """

    # Animation.
    # Idea2: all cubes are moved to center but scaled and thus pushed back!
    # PROBLEM: translation and rotation can not be directly added to rigid body and kinematic body is problematic!
    """
    n_phases = 10
    curr_frame = 30
    frame_delta = 30
    for i in range(n_phases):
        for working_cube in working_cubes:
            if mathutils.noise.random() < 0.5:
                # Random translation to center.
                # Usually working directly with transformation while rigid body is applied on this body is not recommended.
                #world_translation = working_cube.matrix_basis.to_translation()
                #working_cube.location -= working_cube.location # pull towards center
                #working_cube.keyframe_insert("location", frame=curr_frame)
                # Random scale
                scale1 = mathutils.noise.random() * 3.0 + 0.5
                scale2 = mathutils.noise.random() * 3.0 + 0.5
                scale3 = mathutils.noise.random() * 3.0 + 0.5
                tmp_scale = working_cube.scale 
                working_cube.scale = mathutils.Vector((tmp_scale[0], tmp_scale[1], scale1))
                working_cube.keyframe_insert("scale", frame=curr_frame)
                # Random rotate.
                #eul = mathutils.Euler((0.0, math.radians(300.0), 0.0), 'XYZ')
                #working_cube.rotation_euler = eul
                #working_cube.keyframe_insert("rotation_euler", frame=curr_frame)
        curr_frame += frame_delta
    """

    # Animation.
    # Cubes are scaled uniformly once under rigid body constraint.
    """
    curr_frame = 100
    for working_cube in working_cubes:
        # Random scale
        scale = mathutils.noise.random() * 5.0 + 1.0
        working_cube.scale = mathutils.Vector((scale, scale, scale))
        working_cube.keyframe_insert("scale", frame=curr_frame)
    """

    # Animation.
    # Cubes are scaled uniformly multiple times under rigid body constraint.
    """
    n_phases = 10
    frame_delta = 100
    curr_frame = 0 + frame_delta
    for i in range(n_phases):
        for working_cube in working_cubes:
            if mathutils.noise.random() < 0.5:
                # Random scale
                scale1 = mathutils.noise.random() * 3.0 + 0.5
                working_cube.scale = mathutils.Vector((scale1, scale1, scale1))
                working_cube.keyframe_insert("scale", frame=curr_frame)
        curr_frame += frame_delta
    """

    # Animation.
    # Cubes are scaled non-uniformly.
    n_phases = 10
    curr_frame = 30
    frame_delta = 30
    for i in range(n_phases):
        for working_cube in working_cubes:
            if mathutils.noise.random() < 0.5:
                scale1 = mathutils.noise.random() * 3.0 + 0.5
                scale2 = mathutils.noise.random() * 3.0 + 0.5
                scale3 = mathutils.noise.random() * 3.0 + 0.5
                tmp_scale = working_cube.scale 
                working_cube.scale = mathutils.Vector((tmp_scale[0], scale1, tmp_scale[2]))
                working_cube.keyframe_insert("scale", frame=curr_frame)
        curr_frame += frame_delta

    """
    # Animation.
    # Cubes are scaled using noise/sin.
    n_phases = 10
    curr_frame = 30
    frame_delta = 30
    for i in range(n_phases):
        for working_cube in working_cubes:
            # mathutils.noise.fractal(position, H, lacunarity, octaves, noise_basis='PERLIN_ORIGINAL')
            scale1 = mathutils.noise.fractal(working_cube.location + working_cube.scale, 2, 2, 4, noise_basis='PERLIN_ORIGINAL') * 3.3
            working_cube.scale = mathutils.Vector((scale1, scale1, scale1))
            working_cube.keyframe_insert("scale", frame=curr_frame)
        curr_frame += frame_delta
    """


    


if __name__ == "__main__":
    main()
