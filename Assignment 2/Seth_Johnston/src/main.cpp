#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/io.hpp>
#include <vector>
#include <iostream>
#include "MatrixStack.h"
#include "Program.h"

using namespace std;

#define WINDOW_WIDTH 800
#define WINDOW_HEIGHT 800

char* vertShaderPath = "../shaders/shader.vert";
char* fragShaderPath = "../shaders/shader.frag";

GLFWwindow *window;
double currentXpos, currentYpos;
glm::vec3 eye(0.0f, 0.0f, 8.0f);
glm::vec3 center(0.0f, 0.0f, 0.0f); // center = eye - w
glm::vec3 up(0.0f, 1.0f, 0.0f);  // global up vector we rotate 
glm::vec3 newup(0.0f, 1.0f, 0.0f);  // global up vector we rotate 
glm::vec3 u(0.0f, 0.0f, 0.0f);
glm::vec3 newu(0.0f, 0.0f, 0.0f);
glm::vec3 w(0.0f, 1.0f, 0.0f); // w = eye - center; corresponds to a; utilized when zooming in and out

Program program;
MatrixStack modelViewProjectionMatrix;

// Draw cube on screen
void DrawCube(glm::mat4& modelViewProjectionMatrix)
{
	program.SendUniformData(modelViewProjectionMatrix, "mvp");
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

class RobotElements
{
private:

public:
	RobotElements() {}
	~RobotElements() {}

	// Member variables required for the tree hierarchy; needs to be public to access with dot
	vector<RobotElements*> children;

	// A glm::vec3 representing the translation of the component’s joint with respect to the parent component’s joint
	glm::vec3 transComp2Par = glm::vec3(0.0f, 0.0f, 0.0f);

	// A glm::vec3 representing the translation of the component with respect to its joint
	glm::vec3 transComp = glm::vec3(0.0f, 0.0f, 0.0f);

	// A glm::vec3 representing the current joint angles about the X, Y, and Z axes of the component’s joint. (You may want to start with Z-rotations only.
	glm::vec3 compAngles = glm::vec3(0.0f, 0.0f, 0.0f);

	//  A glm::vec3 representing the X, Y, and Z scaling factors for the component
	glm::vec3 compScale = glm::vec3(0.0f, 0.0f, 0.0f);

	// A glm::vec3 representing 
	glm::vec3 selectScale = glm::vec3(0.0f, 0.0f, 0.0f);

	// represents the new rotation that i am trying to give to the specific robot element
	glm::vec3 newRotation = glm::vec3(0.0f, 0.0f, 0.0f);

	// A member method for drawing itself and its children. 
	void elementDraw() {
		// 1. push a matrix to the matrix stack  --> whatever transformations we do for the current element only applies to the current element and not the parents
		modelViewProjectionMatrix.pushMatrix();  // copy of the top
		
		// 2. Apply global tranformations (for yourself and your children)  --> global transformations are passed along to the children
		// rotation and translation with respect to parent are global --> transComp2Par, how much to globally rotate the parent
		// translation defines the joint becasue once you transalte the joints becomes at the center of the coordinate system
		modelViewProjectionMatrix.translate(this->transComp2Par);
		modelViewProjectionMatrix.rotateX(glm::radians(this->compAngles.x + this->newRotation.x));
		modelViewProjectionMatrix.rotateY(glm::radians(this->compAngles.y + this->newRotation.y));
		modelViewProjectionMatrix.rotateZ(glm::radians(this->compAngles.z + this->newRotation.z));

		// 3. draw the chidren (call the same function recursivesly with child.elementDraw())
			// loop through the children of torso, and each of those elements has elementDraw called on all of their children
			for (int i = 0; i < this->children.size(); i++) {
				
				this->children[i]->elementDraw();
			}
		
		// 4. apply local transformations --> shouldn't be applied to the children which is why it occurs after the recursion ends
		// scaling and translation to joint are local  --> transComp, compScale, 
		modelViewProjectionMatrix.translate(this->transComp);
		// if (traversal[vectorPosition] == this) {
		// 	modelViewProjectionMatrix.scale(1.2); // scales selected component -- test a good value
		// }
		modelViewProjectionMatrix.scale(this->compScale.x + this->selectScale.x, this->compScale.y + this->selectScale.y, this->compScale.z + this->selectScale.z);

		// 5. draw current robotElement - each RobotElements object just draws a cube and applies a few transformations on it to become one of the appendages
		DrawCube(modelViewProjectionMatrix.topMatrix());
		// 6. pop matrix - final pop: for every push there is a pop
		modelViewProjectionMatrix.popMatrix();
		// return at the end after all recursion is over
		return;
	}
};

// define Root of tree: always the same value (torso), no matter the specific instance of RobotElements.
RobotElements *root = nullptr;

// 1D Vector to traverse tree
vector<RobotElements*> traversal;
// used in characterCallback
int vectorPosition = 0;

// populate the traversal vector properly with recursive DFS method
void traverse(RobotElements* newroot) {
	traversal.push_back(newroot);
	
	for (int i = 0; i < newroot->children.size(); i++) {
		traverse(newroot->children[i]);

	}
}
   
// construct the robot here essentially a tree structure of RobotElement objects 
void ConstructRobot()
{
	// Define the components of the robot globally
	// everything bracnhes from the torso
	RobotElements *torso = new RobotElements();  // at the root of the tree hierarchy
	RobotElements *head = new RobotElements();
	RobotElements *upperLeftArm = new RobotElements();
	RobotElements *lowerLeftArm = new RobotElements();
	RobotElements *upperRightArm = new RobotElements();
	RobotElements *lowerRightArm = new RobotElements();
	RobotElements *upperLeftLeg = new RobotElements();
	RobotElements *lowerLeftLeg = new RobotElements();
	RobotElements *upperRightLeg = new RobotElements();
	RobotElements *lowerRightLeg = new RobotElements();


	// now assign root to torso; must be done inside this function becasue if assigned at the top outside of the function this compiles before that and it will not work properly
	::root = torso;

	// establish hierarchy
	upperLeftArm->children = {lowerLeftArm};
	upperRightArm->children = {lowerRightArm};
	upperLeftLeg->children = {lowerLeftLeg};
	upperRightLeg->children = {lowerRightLeg};
	torso->children = {upperLeftArm, upperRightArm, upperLeftLeg, upperRightLeg, head};

	// vec3 float assignments for each RobotElement so the program knows important information for transforming each individually on the screen
	torso->transComp2Par = glm::vec3(0.0f, 0.0f, 0.0f);
	torso->transComp = glm::vec3(0.0f, 0.0f, 0.0f);
	torso->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	torso->compScale = glm::vec3(0.6f, 1.4f, 0.6f);

	head->transComp2Par = glm::vec3(0.0f, 1.35f, 0.0f);
	head->transComp = glm::vec3(0.0f, 0.3f, 0.0f);
	head->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	head->compScale = glm::vec3(0.3f, 0.3f, 0.3f);

	upperLeftArm->transComp2Par = glm::vec3(-0.5f, 1.0f, 0.0f);
	upperLeftArm->transComp = glm::vec3(-0.7f, 0.0f, 0.0f);
	upperLeftArm->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	upperLeftArm->compScale = glm::vec3(0.6f, 0.25f, 0.25f);

	lowerLeftArm->transComp2Par = glm::vec3(-1.3f, 0.0f, 0.0f);
	lowerLeftArm->transComp = glm::vec3(-0.5f, 0.0f, 0.0f);
	lowerLeftArm->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	lowerLeftArm->compScale = glm::vec3(0.5f, 0.17f, 0.17f);

	upperRightArm->transComp2Par = glm::vec3(0.5f, 1.0f, 0.0f);
	upperRightArm->transComp = glm::vec3(0.7f, 0.0f, 0.0f);
	upperRightArm->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	upperRightArm->compScale = glm::vec3(0.6f, 0.25f, 0.25f);

	lowerRightArm->transComp2Par = glm::vec3(1.3f, 0.0f, 0.0f);
	lowerRightArm->transComp = glm::vec3(0.5f, 0.0f, 0.0f);
	lowerRightArm->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	lowerRightArm->compScale = glm::vec3(0.5f, 0.17f, 0.17f);

	upperLeftLeg->transComp2Par = glm::vec3(-0.35f, -1.2f, 0.0f);
	upperLeftLeg->transComp = glm::vec3(0.0f, -0.8f, 0.0f);
	upperLeftLeg->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	upperLeftLeg->compScale = glm::vec3(0.25f, 0.7f, 0.25f);

	lowerLeftLeg->transComp2Par = glm::vec3(0.0f, -1.4f, 0.0f);
	lowerLeftLeg->transComp = glm::vec3(0.0f, -0.7f, 0.0f);
	lowerLeftLeg->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	lowerLeftLeg->compScale = glm::vec3(0.17f, 0.6f, 0.17f);

	upperRightLeg->transComp2Par = glm::vec3(0.35f, -1.2f, 0.0f);
	upperRightLeg->transComp = glm::vec3(0.0f, -0.8f, 0.0f);
	upperRightLeg->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	upperRightLeg->compScale = glm::vec3(0.25f, 0.7f, 0.25f);

	lowerRightLeg->transComp2Par = glm::vec3(0.0f, -1.4f, 0.0f);
	lowerRightLeg->transComp = glm::vec3(0.0f, -0.7f, 0.0f);
	lowerRightLeg->compAngles = glm::vec3(0.0f, 0.0f, 0.0f);
	lowerRightLeg->compScale = glm::vec3(0.17f, 0.6f, 0.17f);

}

//////////// What I need help with/understanding ///////////////

///////////// NOTES ////////////////
// global you pass to children as well and local you only want to do to yourself
// Global = translation with respect to parent and rotation  --> must also pass onto children
// local = translation with respect to current joint and scaling  (we only wan to scale a single component)

// start with zooming in with camera:
// a vector = eye - center  -- corresponds to w
// right vector correspnds to u
// closer to object = 0.9 x a
// eye = center + new a value --> update eye value
// glm::rotate(angle, axis) which you multiply with a which will give the rotated w, then compute the new eye value which is eye^r = center + W^r

// Responsible for drawing the cubes on the screen; sets the transformations using the 
// global matrix stack
void Display()
{	
	
	program.Bind();
	// identity at the base of the tree
	// modelViewProjectionMAtrix is the global matrix stack variable
	modelViewProjectionMatrix.loadIdentity();
	modelViewProjectionMatrix.pushMatrix();

	// Setting the view and Projection matrices
	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	// perspective transform
	modelViewProjectionMatrix.Perspective(glm::radians(60.0f), float(width) / float(height), 0.1f, 100.0f);
	// view transform
	modelViewProjectionMatrix.LookAt(eye, center, up);
	
	// Call elementDraw after the model transformations on the root element
	root->elementDraw();

	// final pop: for every push there is a pop
	modelViewProjectionMatrix.popMatrix();

	program.Unbind();
	
}

// Mouse callback function
void MouseCallback(GLFWwindow* lWindow, int button, int action, int mods)
{
	if (button == GLFW_MOUSE_BUTTON_LEFT && GLFW_PRESS == action)
		std::cout << "Mouse left button is pressed." << std::endl;
}

// The callback function receives two-dimensional scroll offsets.
// utilizing yoffset values to scroll in and out when zooming in and out via going up and down on touchpad
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) 
{		
	w.x = eye.x - center.x;
	w.y = eye.y - center.y;
	w.z = eye.z - center.z;
	w.x = w.x + (yoffset * w.x)/100;
	w.y = w.y + (yoffset * w.y)/100;
	w.z = w.z + (yoffset * w.z)/100;
	eye.x = center.x + w.x;
	eye.y = center.y + w.y;
	eye.z = center.z + w.z;
}

// Mouse position callback function
double prevxpos = 0.0;
double prevypos = 0.0;
void CursorPositionCallback(GLFWwindow* lWindow, double xpos, double ypos)
{

	// You should use rotation matrices for both horizontal and vertical rotations. glm::rotate takes an angle and a vector 
	// (the axis around which the rotation happens) and creates a rotation matrix. So all you need to do is, to pass the angle and appropriate vector  
	// (global up for horizontal and local right for vertical) and angle to this function to get the matrices. Then you multiply the matrices with a (from slides) and up. The updated a is used 
	// to generate the new eye position. The updated up is used as the current up.



	// The translation should look like the robot is moving to the opposite direction of the camera translation.
	// For example, when moving the camera to the right, the robot will move to the left. This happens when you add or subtract to the eye and the center of the camera.
	// For rotation, you should implement it in relation to the global center point (0,0,0).
	// Moving the mouse horizontally should rotate the camera around the global up vector (without modifying the distance to the robot a). The effect is a horizontal 360 view of the robot, showing its front, passing to its side, back, and back to the front.
	// Moving the mouse vertically should rotate the camera around the local x axis (right vector of the camera), which you can compute from the up and eye vectors. 
	int state = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
	int state2 = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
	if (state == GLFW_PRESS) {
		std::cout << "Left Mouse position is: x - " << xpos << ", y - " << ypos << std::endl;

		w.x = eye.x - center.x;
		w.y = eye.y - center.y;
		w.z = eye.z - center.z;


		// rotation around global z axis
		glm::mat3 rotationMatrix1(1.0f);
		glm::mat3 rotationMatrix2(1.0f);
		float calc1 =  0.5 * (xpos - ::prevxpos);
		float calc2 =  0.5 * (ypos - ::prevypos);
		u = glm::cross(up, w) / glm::length(glm::cross(up, w));
		rotationMatrix1 = glm::rotate(glm::mat4(1.0f), glm::radians(calc1), up);
		rotationMatrix2 = glm::rotate(glm::mat4(1.0f), glm::radians(calc2), u);
		
		// w vector multiplication with itself and the rotation matrices
		w = rotationMatrix1 * rotationMatrix2 * w;


		// eye^r
		eye.x = w.x + center.x;
		eye.y = w.y + center.y;
		eye.z = w.z + center.z;

		// The up vector needs to be updated, which is the cross product of the normalized eye and normalized right vector
		up = glm::cross(eye, u) / glm::length(glm::cross(eye, u));

	} else if (state2 == GLFW_PRESS) {
		std::cout << "Right Mouse position is: x - " << xpos << ", y - " << ypos << std::endl;

		float calc1 =  0.01 * (xpos - ::prevxpos);
		float calc2 =  0.01 * (ypos - ::prevypos);

		center.x = eye.x - w.x + calc1;
		center.y = eye.y - w.y + calc2;
		center.z = eye.z - w.z;

		w.x = eye.x - center.x;
		w.y = eye.y - center.y;
		w.z = eye.z - center.z;

		
	}

	::prevxpos = xpos;
	::prevypos = ypos;
}

// Keyboard character callback function
void CharacterCallback(GLFWwindow* lWindow, unsigned int key)
{
	std::cout << "Key " << (char)key << " is pressed." << std::endl;

	switch(char(key)) {
	// traverse forward through hierarchy
	case '.':
		(traversal[vectorPosition])->selectScale.x = 0.0f;
		(traversal[vectorPosition])->selectScale.y = 0.0f;
		(traversal[vectorPosition])->selectScale.z = 0.0f;
		vectorPosition = (vectorPosition + 1) % traversal.size();
		(traversal[vectorPosition])->selectScale.x = 0.025f;
		(traversal[vectorPosition])->selectScale.y = 0.025f;
		(traversal[vectorPosition])->selectScale.z = 0.025f;
		break;
	// traverse backward through hierarchy
	case ',':
		(traversal[vectorPosition])->selectScale.x = 0.0f;
		(traversal[vectorPosition])->selectScale.y = 0.0f;
		(traversal[vectorPosition])->selectScale.z = 0.0f;
		vectorPosition = !vectorPosition ? traversal.size()-1 : vectorPosition - 1;
		(traversal[vectorPosition])->selectScale.x = 0.025f;
		(traversal[vectorPosition])->selectScale.y = 0.025f;
		(traversal[vectorPosition])->selectScale.z = 0.025f;
		break;
	// Increment x Angle
	case 'x':
		traversal[vectorPosition]->newRotation = {traversal[vectorPosition]->newRotation.x + 4,
		 traversal[vectorPosition]->newRotation.y,
		  traversal[vectorPosition]->newRotation.z};
		  break;
	// Decrement x Angle
	case 'X':
		traversal[vectorPosition]->newRotation = {traversal[vectorPosition]->newRotation.x - 4,
		 traversal[vectorPosition]->newRotation.y,
		  traversal[vectorPosition]->newRotation.z};
		break;
	// Increment y Angle
	case 'y':
		traversal[vectorPosition]->newRotation = {traversal[vectorPosition]->newRotation.x,
		  traversal[vectorPosition]->newRotation.y + 4,
			traversal[vectorPosition]->newRotation.z};
		break;
	// Decrement y Angle
	case 'Y':
		traversal[vectorPosition]->newRotation = {traversal[vectorPosition]->newRotation.x,
		 traversal[vectorPosition]->newRotation.y - 4,
		  traversal[vectorPosition]->newRotation.z};
		break;
	// Increment z Angle
	case 'z':
		traversal[vectorPosition]->newRotation = {traversal[vectorPosition]->newRotation.x,
		 traversal[vectorPosition]->newRotation.y,
		  traversal[vectorPosition]->newRotation.z + 4};
		break;
	// Decrement z Angle
	case 'Z':
		traversal[vectorPosition]->newRotation = {traversal[vectorPosition]->newRotation.x,
		 traversal[vectorPosition]->newRotation.y,
		  traversal[vectorPosition]->newRotation.z - 4};
		break;
	default:
		cout << "Nothing happened" << endl;
		break;
	}
}


void CreateCube()
{
	// x, y, z, r, g, b, ...
	float cubeVerts[] = {
		// Face x-
		-1.0f,	+1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	+1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	-1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	-1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	+1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		-1.0f,	-1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		// Face x+
		+1.0f,	+1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	-1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	+1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	+1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	-1.0f,	+1.0f,	0.8f,	0.2f,	0.2f,
		+1.0f,	-1.0f,	-1.0f,	0.8f,	0.2f,	0.2f,
		// Face y-
		+1.0f,	-1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	-1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		+1.0f,	-1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		+1.0f,	-1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	-1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	-1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		// Face y+
		+1.0f,	+1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		+1.0f,	+1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	+1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	+1.0f,	+1.0f,	0.2f,	0.8f,	0.2f,
		+1.0f,	+1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		-1.0f,	+1.0f,	-1.0f,	0.2f,	0.8f,	0.2f,
		// Face z-
		+1.0f,	+1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		+1.0f,	-1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	+1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	+1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		+1.0f,	-1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	-1.0f,	-1.0f,	0.2f,	0.2f,	0.8f,
		// Face z+
		+1.0f,	+1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	+1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		+1.0f,	-1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		+1.0f,	-1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	+1.0f,	+1.0f,	0.2f,	0.2f,	0.8f,
		-1.0f,	-1.0f,	+1.0f,	0.2f,	0.2f,	0.8f
	};

	GLuint vertBufferID;
	glGenBuffers(1, &vertBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, vertBufferID);
	glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVerts), cubeVerts, GL_STATIC_DRAW);
	GLint posID = glGetAttribLocation(program.GetPID(), "position");
	glEnableVertexAttribArray(posID);
	glVertexAttribPointer(posID, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), 0);
	GLint colID = glGetAttribLocation(program.GetPID(), "color");
	glEnableVertexAttribArray(colID);
	glVertexAttribPointer(colID, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));

}

void FrameBufferSizeCallback(GLFWwindow* lWindow, int width, int height)
{
	glViewport(0, 0, width, height);
}

void Init()
{
	glfwInit();
	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Assignment2 - Seth Johnston", NULL, NULL);
	glfwMakeContextCurrent(window);
	glewExperimental = GL_TRUE;
	glewInit();
	glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
	glfwSetMouseButtonCallback(window, MouseCallback);
	glfwSetCursorPosCallback(window, CursorPositionCallback);
	glfwSetScrollCallback(window, scroll_callback);
	glfwSetCharCallback(window, CharacterCallback);
	glfwSetFramebufferSizeCallback(window, FrameBufferSizeCallback);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);

	program.SetShadersFileName(vertShaderPath, fragShaderPath);
	program.Init();

	CreateCube();


	ConstructRobot();

	// only need to establish the recursively traveled tree once so we do it inside of Init(); must be called after ConstructRobot()
	traverse(root);

	// glfwGetTime();
}


int main()
{	
	Init();
	while ( glfwWindowShouldClose(window) == 0) 
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		Display();
		glFlush();
		glfwSwapBuffers(window);
		glfwPollEvents();
		// glfwSetTime();
			

	}

	glfwTerminate();
	return 0;
}