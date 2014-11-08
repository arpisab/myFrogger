////////////////////////////////////////////////////////////////////////////////
//
// (C) Andy Thomason 2012-2014
//
// Modular Framework for OpenGLES2 rendering on multiple platforms.
//

////\\_//\\_////==->


//#include "myFrog.h"

namespace octet {

	//// Scene containing a box with octet.
	class myFrogger : public app {
		//// scene for drawing box 
		ref<visual_scene> app_scene;

		btDefaultCollisionConfiguration config;       /// setup for the world
		btCollisionDispatcher *dispatcher;            /// handler for collisions between objects
		btDbvtBroadphase *broadphase;                 /// handler for broadphase (rough) collision
		btSequentialImpulseConstraintSolver *solver;  /// handler to resolve collisions
		btDiscreteDynamicsWorld *world;             /// physics world, contains rigid bodies

		dynarray<btRigidBody*> rigid_bodies;
		dynarray<scene_node*> nodes;

		/// this function is called to initialize the Frog's mesh //////////////////////////////



		/// This takes mesh and materials from Collada and put them in the Rigid Body
		void add_ColladaShape(scene_node *_node, mesh *_mesh, material *_material, bool is_dynamic, btScalar mass_in, string rbShape = "Box") {

			aabb aabbMesh = _mesh->get_aabb(); /// the aabb createts a bounding box for the collada mesh
			vec3 size = aabbMesh.get_half_extent(); /// the vec3 mesure the size of the 

			btCollisionShape *C_shape = new btBoxShape(get_btVector3(size)); // box is default shape

			if (rbShape == "Cylinder") {                                               ///
				C_shape = new btCylinderShape(get_btVector3(size));	    ///
			}
			else if (rbShape == "Sphere") {
				C_shape = new btSphereShape(size.x());
			}
			
			//btCollisionShape *C_shape = new btCylinderShape(get_btVector3(size));
			btScalar mass = is_dynamic ? mass_in : 0.0f;
			btVector3 inertiaTensor;
			C_shape->calculateLocalInertia(mass, inertiaTensor);

			mat4t modelToWorld = _node->access_nodeToParent();	  /// this creates the 4x4 matrix modelToWorld
			btMatrix3x3 matrix(get_btMatrix3x3(modelToWorld));		/// this passess the rotation and scale matrix 3*3 to the mat4t (modelToWorld)
			btVector3 pos(get_btVector3(modelToWorld[3].xyz()));	/// this passes the xyz translate vector to the modelToWorld
			btTransform transform(matrix, pos);				    /// this creates the bullet physics matris "transform" and we pass in the matrix and the vector
			btDefaultMotionState *motionState = new btDefaultMotionState(transform);        /// this 

			btRigidBody *rigidbody = new btRigidBody(mass, motionState, C_shape, inertiaTensor);    /// here we create the rigid body and pass the necessary parameters to build up the node
			world->addRigidBody(rigidbody);															/// this create the instance of the rigibody and adds the rigid body in it
			rigid_bodies.push_back(rigidbody);														/// this push the instance of the rigid body with the rigid body to the world

			nodes.push_back(_node);																	/// 
			app_scene->add_child(_node);															/// 
			app_scene->add_mesh_instance(new mesh_instance(_node, _mesh, _material));				/// 
		}

		
		/// this function is called to initialize primitive meshes //////////////////////////////
		void add_shape(mat4t_in mat, mesh *msh, material *mtl, bool is_dynamic) {
			scene_node *node = new scene_node();
			node->access_nodeToParent() = mat;
			app_scene->add_child(node);
			app_scene->add_mesh_instance(new mesh_instance(node, msh, mtl));
			nodes.push_back(node);

			btMatrix3x3 matrix(get_btMatrix3x3(mat));
			btVector3 pos(get_btVector3(mat[3].xyz()));

			btCollisionShape *shape = is_dynamic ? msh->get_bullet_shape() : msh->get_static_bullet_shape();
				
			if (shape) {
				btTransform transform(matrix, pos);

				btDefaultMotionState *motionState = new btDefaultMotionState(transform);
				btScalar mass = is_dynamic ? 1.0f : 0.0f;
				btVector3 inertiaTensor;

				if (is_dynamic) shape->calculateLocalInertia(mass, inertiaTensor);

				btRigidBody * rigid_body = new btRigidBody(mass, motionState, shape, inertiaTensor);
				world->addRigidBody(rigid_body);
				rigid_body->setUserPointer(node);
				rigid_bodies.push_back(rigid_body);
			}
		}



		scene_node *camera_node;

	/// this is called once OpenGL is initialized 
	public:
		myFrogger(int argc, char **argv) : app(argc, argv) {
			dispatcher = new btCollisionDispatcher(&config);
			broadphase = new btDbvtBroadphase();
			solver = new btSequentialImpulseConstraintSolver();
			world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, &config);
		}

		~myFrogger() {
			delete world;
			delete solver;
			delete broadphase;
			delete dispatcher;
		}

		/// this is called once OpenGL is initialized 
		void app_init() {
			app_scene = new visual_scene();
			app_scene->create_default_camera_and_lights();
			
			mat4t mat;

			//////////////////////////////// Light
			scene_node *light_node = new scene_node();
			light *_light = new light();
			//_light->set_kind(atom_directional);
			_light->set_attenuation(1, 0, 0);
			_light->set_color(vec4(1, 1, 1, 1));
			light_node->rotate(-45, vec3(1, 0, 0));
			light_node->translate(vec3(0, 0, 100));
			app_scene->add_light_instance(new light_instance(light_node, _light));

			//////////////////////////////// Camera 
			camera_node = app_scene->get_camera_instance(0)->get_node();
			app_scene->get_camera_instance(0)->set_far_plane(10000.0f);
			//camera_node->translate(vec3(0, 50, 50));      /// (0, 50, 50)
			//camera_node->rotate(-40.0f, vec3(1, 0, 0));   /// (1, 0, 0)

			
								
			
			
			//////////////////////////////// Ground 
			
			mat.translate(0, -1, 0);
			material *floor_mat = new material(vec4(1, 0, 1, 1));
			image *sabImage = new image("assets/myFrogger/sabTxt.jpg");
			material *sabMat = new material(sabImage);
			add_shape(mat, new mesh_box(vec3(15000.0f, 0.5f, 10000.0f)), sabMat, false);   /// (150.0f, 0.5f, 100.0f

			//////////////////////////////// myFrog 
				
				
			/*
			mat4t objec2World;
			objec2World.loadIdentity();
			objec2World.translate(0, 10, 0);
			my_Frog.init_sphere(objec2World, 0.5f, red, 1.0f);
			my_Frog.add_to_scene(nodes, app_scene, *world, rigid_bodies);
			*/


			//////////////////////////////// Collada builder
			mat4t modelToWorld;
			collada_builder colladaLoader;
			resource_dict dict;
			dynarray<resource*> colladaMeshes;
			dynarray<resource*> colladaMaterials;
			

			//////////////////////////////// load meshes
			/// this load the leaf mesh
			/*if (!colladaLoader.load_xml("assets/myfrogger/Leaf01.dae")) {
				printf("failed to load file");
				return;
			}*/
			
			/* this loads first car coming from Left
			if (!colladaLoader.load_xml("assets/myfrogger/LCar01.dae")) {
				printf("failed to load file");
				return;
			}*/
			
			/// this loads first car coming from Right
			if (!colladaLoader.load_xml("assets/myfrogger/myCar01.dae")) {
				printf("failed to load file");
				return;
			}

			colladaLoader.get_resources(dict);
			dict.find_all(colladaMeshes, atom_mesh);
			dict.find_all(colladaMeshes, atom_material);
			printf("collada_meshes size: %i\n", colladaMeshes.size());
			printf("collada_materials size: %i\n", colladaMaterials.size());

				

			/// Materials
			material *RCarTxt = new material(new image("assets/myFrogger/TreeTrunk.gif"));
			material *LCarTxt = new material(new image("assets/myFrogger/TreeTrunk.gif"));
			material *Leaf01 = new material(new image("assets/myFrogger/TreeTrunk.gif"));
			
				
			/// car

			/*mesh *leafMesh;
			if (colladaMeshes.size()) {
				leafMesh = colladaMeshes[0]->get_mesh();
				scene_node *leafNode = new scene_node();
				leafNode->translate(vec3(0, 10, 0));
				add_ColladaShape(leafNode, leafMesh, LCarTxt, "box", 1.0f);
			}*/

			mesh *LCarMesh;
			if (colladaMeshes.size()) {
				LCarMesh = colladaMeshes[0]->get_mesh();
				scene_node *LCarNode = new scene_node();
				LCarNode->translate(vec3(0, 20, 0));
				add_ColladaShape(LCarNode, LCarMesh, LCarTxt, true, 1.0f, "box");
			}

			/*mesh *RCarMesh;
			if (colladaMeshes.size()) {
				RCarMesh = colladaMeshes[1]->get_mesh();
				scene_node *RCarNode = new scene_node();
				RCarNode->translate(vec3(0, 10, 0));
				add_ColladaShape(RCarNode, RCarMesh, LCarTxt, "box", 1.0f);
			}*/



			//////////////////////////////// Materials 
			material *red = new material(vec4(1, 0, 0, 1));
			material *green = new material(vec4(0, 1, 0, 1));
			material *blue = new material(vec4(0, 0, 1, 1));

			
			
			//////////////////////////////// Primitives 
			mat.loadIdentity();
			mat.translate(-10, 3, 0);
			mat.rotateX(90);
			
			add_shape(mat, new mesh_cylinder(zcylinder(vec3(0, 0, 0), 6, 3)), green, true);
			
			mat.translate(3, 3, 0);
			add_shape(mat, new mesh_cylinder(zcylinder(vec3(0, 0, 0), 6, 3)), blue, true);





		}

		/// this is called to draw the world 
		void draw_world(int x, int y, int w, int h) {
				int vx = 0, vy = 0;
				get_viewport_size(vx, vy);
				app_scene->begin_render(vx, vy);

				world->stepSimulation(1.0f / 30, 1, 1.0f / 30);

				for (unsigned i = 0; i != rigid_bodies.size(); ++i) {
					btRigidBody *rigid_body = rigid_bodies[i];
					btQuaternion btq = rigid_body->getOrientation();
					btVector3 pos = rigid_body->getCenterOfMassPosition();
					quat q(btq[0], btq[1], btq[2], btq[3]);
					if (i == 1){
						//source - ryan alexander singh
						scene_node *cameraNode = app_scene->get_camera_instance(0)->get_node();
						nodes[1]->add_child(cameraNode);
						mat4t &cameraMatrix = cameraNode->access_nodeToParent();
						cameraNode->loadIdentity();
						camera_node->translate(vec3(0, 50, 50));      /// (0, 50, 50)
						camera_node->rotate(-40.0f, vec3(1, 0, 0));   /// (1, 0, 0)
						//cameraMatrix.translate(0, 100, 0);   //(0, 10, -20)

						//source end
					}
					mat4t modelToWorld = q;
					modelToWorld[3] = vec4(pos[0], pos[1], pos[2], 1);
					nodes[i]->access_nodeToParent() = modelToWorld;
				
				
				}
				
				//////////////////////////////// Frog CTRL ////////////////////////////////
				if (is_key_down(key_up)){
					rigid_bodies[1]->applyCentralImpulse(btVector3(0.0f, 1.0f, 0.0f));
				}

				////////////////////////////////  Camera CTRL ////////////////////////////////
				if (is_key_down('W')) {
					camera_node->translate(vec3(0, 0, -3.0f));	// go foward
				}
				else if (is_key_down('S')) {
					camera_node->translate(vec3(0, 0, 3.0f));	// go back
				}
				else if (is_key_down('E')) {
					camera_node->translate(vec3(0, 3.0f, 0));	// go up
				}
				else if (is_key_down('Q')) {
					camera_node->translate(vec3(0, -3.0f, 0));	// go down
				}
				else if (is_key_down('A')) {
					camera_node->rotate(1.0f, vec3(0, 1, 0));	// go left
				}
				else if (is_key_down('D')) {
					camera_node->rotate(-1.0f, vec3(0, 1, 0));	// go right
				}
				else if (is_key_down('C')) {
					camera_node->rotate(-1.0f, vec3(1, 0, 0));	// go right
				}
				else if (is_key_down('Z')) {
					camera_node->rotate(1.0f, vec3(1, 0, 0));	// go right
				}

				// update matrices. assume 30 fps.
				app_scene->update(1.0f / 30);

				// draw the scene
				app_scene->render((float)vx / vy);
			}
	};

}