#include "Arduino.h"

#include <Servo.h>
#include <math.h>

int sensorPin = 3;  //A3 for sensor

//motors
int leftFrontPin = 11;
int rightFrontPin = 10;
int headPin = 9;
int leftBackPin = 6;
int rightBackPin = 5;

Servo leftFrontLegServo;
Servo rightFrontLegServo; 
Servo headServo;
Servo leftBackLegServo;  
Servo rightBackLegServo; 

int object_value = 184;

// for T-positon
int pos1, pos2, pos3, pos4, pos5;

// counter for walking
double counter;

// definition of Action
typedef enum {
  forward,
  backward,
  left,
  right,
} Body_Action;

Body_Action body_action;

// definition of Action
typedef enum {
  straigt,
  look_left,
  look_right,
} Head_Action;

Head_Action head_action;

// definition of Situation
typedef enum {
  normal_left,
  normal_straigt,
  normal_right,
  danger_left,
  danger_straigt,
  danger_right,
} Body_Situation;

Body_Situation body_situation;

// definition of Situation
typedef enum {
  closer,
  same,
  away,
} Head_Situation;

Head_Situation head_situation;

//ml body
int body_epsilon = 95;   // 1 - epsilon in %
float body_learning_rate = 0.89;
float body_discount_factor = 0.8;

// situation x action
float body_q[6][4] = {{0, 0, 0, 0,},
  {0, 0, 0, 0,},
  {0, 0, 0, 0,},
  {0, 0, 0, 0,},
  {0, 0, 0, 0,},
  {0, 0, 0, 0,},
};

float body_r[6][4] = {{0, -2, 0, 0,},
  {0, -2, 0, 0,},
  {0, -2, 0, 0,},
  {-10, -11, -10, -10,},
  {-10, -11, -10, -10,},
  {-10, -11, -10, -10,},
};

//ml head
int head_epsilon = 85;   // 1 - epsilon in %
float head_learning_rate = 0.6;
float head_discount_factor = 0.6;

// situation x (action + 1 for using in functions)
float head_q[3][4] = {{0, 0, 0, 0,},
  {0, 0, 0, 0,},
  {0, 0, 0, 0,},
};

float head_r[3][4] = {{1, 1, 1, 0,},
  {0, 0, 0, 0,},
  {-1, -1, -1, 0,},
};


void setup() {

  Serial.begin(9600);

  // connect servo with pins
  leftFrontLegServo.attach(leftFrontPin);  
  rightFrontLegServo.attach(rightFrontPin); 
  headServo.attach(headPin); 
  leftBackLegServo.attach(leftBackPin); 
  rightBackLegServo.attach(rightBackPin);

  // set counter for movement methodes
  counter = 0;

  //set values for T pose
  pos1=90;   // leg left front
  pos2=120;   // leg right front
  pos3=110;   // sensor head
  pos4 =20;  // leg left back
  pos5 =50; // leg right back

  // makes startup T-postion
  leftFrontLegServo.write(pos1);
  rightFrontLegServo.write(pos2);
  headServo.write(pos3);
  leftBackLegServo.write(pos4);
  rightBackLegServo.write(pos5);

  // init Head Situation
  head_situation = same;

  // init body Situation
  //body_situation = observe();
  
}

// interface for body action
void take_Body_Action (Body_Action a) {
  switch(a) {
    case right: turn(-0.002); break;
    case backward: go(-0.002); break;
    case left: turn(0.002); break;
    default: go(0.002); break;
  }
}


// interface for head action
void take_Head_Action (Head_Action a) {
  switch(a) {
    case look_right: turn_head(-30); break;
    case look_left: turn_head(30); break;
    default: turn_head(0); break;
  }
}

// turn sin turning direction, cos no-turning direction, lookat for seeing in direction
void turn(double walking_direction){
  for(int i=0;i<1500;i++){
    double angle1 = 18*sin(counter);
    double angle2 = 18*cos(counter);
    
    leftFrontLegServo.write(pos1+angle1);
    rightFrontLegServo.write(pos2+angle2);
    leftBackLegServo.write(pos4+angle1);
    rightBackLegServo.write(pos5+angle2);
    
    counter=counter+walking_direction;
  }
}

// walks: + forward - backward
void go(double walking_direction){
  for(int i=0;i<3141;i++){
    double angle1 = 18*sin(counter);
    double angle2 = 18*cos(counter);
    
    leftFrontLegServo.write(pos1+angle1);
    rightFrontLegServo.write(pos2+angle1);
    leftBackLegServo.write(pos4+angle2);
    rightBackLegServo.write(pos5+angle2);
    
    counter=counter+walking_direction;
  }
}

// turns head
void turn_head(int change){
  headServo.write(pos3 + change);
}

// observe for body (moves head)
Body_Situation observe(){
  // gets distance
  int start_distance = analogRead(sensorPin);
  
  // desides head action
  head_action = make_decision_epsilon_greedy(head_situation, 3, head_q, head_epsilon);
  
  // takes action
  take_Head_Action(head_action);
  
  // get new distance and diffrence
  int new_situation = analogRead(sensorPin);
  int dif = start_distance - new_situation;

  // observes head situation
  Head_Situation new_head_situation;
  if (dif > 0) {
    new_head_situation = away;
  } else if (dif < 0) {
    new_head_situation = closer;
  } else {
    new_head_situation = same;
  }
  
  
  // update head_q
  head_q[head_situation][head_action] = getnewQsa(head_situation, head_action, 3, new_head_situation, head_q, head_r, head_learning_rate, head_discount_factor);

  // updates situation
  head_situation = new_head_situation;  

  // returns body_situation
  if(new_situation < object_value){
    return head_action + 0;
  } else {
    return head_action +3;
  }
  
}

// desides next action
int make_decision_epsilon_greedy(int s, int action_amout, float Q[10][4], int epsilon){
  int r = random(100);
  if(r < epsilon){
    // default is forward
    int a = 0;

    // take action with best Q value for situation s
    for(int i=0; i< action_amout; i++){
      if(Q[s][i]>Q[s][a]){
        a=i;
      }
    }

    return a;

  } else {
    return random(action_amout);
  }
}


// computates new Q(s,a)
float getnewQsa(int s, int a, int action_amout, int s_new, float Q[10][4], float R[10][4], float learning_rate, float discount_factor){
  float maxQ = Q[s_new][0]- Q[s][a];
  for(int i=0; i< action_amout; i++){
    if((Q[s_new][i]- Q[s][a])>maxQ){
      maxQ = Q[s_new][i]- Q[s][a];
    }
  }

  // Q(s,a) = Q(s,a) + learning_reate*(r(s,a)+discount_factor*max to a_new(Q(s_new, a_new) - Q(s,a))
  return Q[s][a] +  learning_rate*(R[s][a]+discount_factor*maxQ);
}

// prints both Q matrix
void output(){
  String message = "";

  // prints head_q
  Serial.println("Q Head:");
  for(int t=0; t< 3; t++){
    message = "";
    for(int v=0; v< 3; v++){
      message = message + "," + static_cast<int> (head_q[t][v]);
    }
    Serial.println(message);
  }
  Serial.println(head_situation);

  // prints body_q
  Serial.println("Q Body:");
  for(int t=0; t< 6; t++){
    message = "";
    for(int v=0; v< 4; v++){
      message = message + "," + static_cast<int> (body_q[t][v]);
    }
    Serial.println(message);
  }
  Serial.println(body_situation);

  Serial.println(analogRead(sensorPin));

  Serial.println("-----");
}


void csv_output() {
  Serial.println("Head");
  String message = "";
  for(int t=0; t< 3; t++){
    message = "";
    for(int v=0; v< 3; v++){
      message = message + "," + static_cast<int> (head_q[t][v]);
    }
    Serial.println(message);
  }
  Serial.println("body");
  for(int t=0; t< 6; t++){
    message = "";
    for(int v=0; v< 4; v++){
      message = message + "," + static_cast<int> (body_q[t][v]);
    }
    Serial.println(message);
  }
}


void loop() {
  // decides body_action
  body_action = make_decision_epsilon_greedy(body_situation, 4, body_q, body_epsilon);
  
  // body takes actions
  take_Body_Action(body_action);

  
  // observes new situation and to head action with learning
  Body_Situation new_body_situation = observe();
    

  
  // update body_q
  body_q[body_situation][body_action] = getnewQsa(body_situation, body_action, 4, new_body_situation, body_q, body_r, body_learning_rate, body_discount_factor);

  // update body_situation
  body_situation = new_body_situation;  

  // prints both Q matirx
  csv_output();
  
}
