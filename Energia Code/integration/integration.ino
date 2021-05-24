/*
 * integration.ino
 * Final sketch for SIXT33N Speech version
 *
 * EE16B Fall 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

/********************************************************/
/********************************************************/
/***                                                  ***/
/*** Constants and global variables from turning.ino. ***/
/***                                                  ***/
/********************************************************/
/********************************************************/

#define LEFT_MOTOR                  P2_0
#define LEFT_ENCODER                P6_6
#define RIGHT_MOTOR                 P1_5
#define RIGHT_ENCODER               P7_0
#define PUSH_START                  PUSH1
int commands[9] = {0, 1, 1, 3, 3, 2, 1, 0, 2};
int dex = 0;


#define SAMPLING_INTERVAL           100
int sample_lens[4] = {0};

// Operation modes
#define MODE_LISTEN                 0
#define MODE_DRIVE                  1
int timer_mode = MODE_LISTEN;

#define DRIVE_FAR                   0
#define DRIVE_LEFT                  1
#define DRIVE_CLOSE                 2
#define DRIVE_RIGHT                 3

#define JOLT_STEPS                  2

boolean loop_mode = MODE_DRIVE;
int drive_mode = 0;

int step_num = 0;
volatile boolean do_loop = 0; // timer signal to increment timestep

typedef struct encoder {
  int pin;
  int pos;
  bool level;
  int avg;
} encoder_t;

encoder_t left_encoder = {LEFT_ENCODER, 0, LOW, 0};
encoder_t right_encoder = {RIGHT_ENCODER, 0, LOW, 0};

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*      From turning.ino     */
/*---------------------------*/

float theta_left = 0.3379;
float theta_right = 0.318;
float beta_left = -11.25;
float beta_right = -11.46;
float v_star = 53.8;

// PWM inputs to jolt the car straight
int left_jolt = 190;
int right_jolt = 200;

// Control gains
float k_left = 0.28;
float k_right = 0.12;

/*---------------------------*/
/*      CODE BLOCK CON2      */
/*      From turning.ino     */
/*---------------------------*/

float driveStraight_left(float delta) {
  return (v_star + beta_left)/theta_left - k_left*(delta/theta_left);
}

float driveStraight_right(float delta) {
  return (v_star + beta_right)/theta_right + k_right*(delta/theta_right);
}
/*---------------------------*/
/*      CODE BLOCK CON3      */
/*      From turning.ino     */
/*---------------------------*/

float delta_ss = 21;

/*---------------------------*/
/*      CODE BLOCK CON4      */
/*      From turning.ino     */
/*---------------------------*/

#define CAR_WIDTH                   15.0 // in cm
#define TURN_RADIUS                 155 // in cm - 6 feet diameter
// #define TURN_RADIUS                 60 // in cm - 4 feet diameter

int run_times[4] = {7000, 5000, 2500, 5000};

float delta_reference(int n) {
  // YOUR CODE HERE
  if (drive_mode == DRIVE_RIGHT) {
    return  (CAR_WIDTH*v_star*n)/(TURN_RADIUS*5);
  }
  else if (drive_mode == DRIVE_LEFT) {
    return -(CAR_WIDTH*v_star*n)/(TURN_RADIUS*5);
  }
  else { // DRIVE_STRAIGHT
    return 0;
  }
}

/*---------------------------*/
/*      CODE BLOCK CON5      */
/*      From turning.ino     */
/*---------------------------*/
#define INFINITY                    (3.4e+38)
#define STRAIGHT_RADIUS             -1

float straight_correction(int n) {
  // YOUR CODE HERE
  return -(CAR_WIDTH*v_star*n)/(TURN_RADIUS*5);
}

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*********************************************************/
/*********************************************************/
/***                                                   ***/
/*** Constants and glboal variables from classify.ino. ***/
/***                                                   ***/
/*********************************************************/
/*********************************************************/

#define MIC_INPUT                   P6_0

#define SIZE                        3200
#define SIZE_AFTER_FILTER           200
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*     From classify.ino     */
/*---------------------------*/

// Enveloping and threshold constants
#define SNIPPET_SIZE                  110
#define PRELENGTH                     15
#define THRESHOLD                     0.3

#define EUCLIDEAN_THRESHOLD         0.1
#define LOUDNESS_THRESHOLD          300

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/


/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*     From classify.ino     */
/*---------------------------*/

float pca_vec1[110] = {0.0010857824141708022, -0.0015911203565807108, 0.0032846705900205775, 0.004038819388261783, 0.0008767300960683538, -0.006568736398010668, 0.0018965450085596275, 0.005226136027051376, 0.002468276863911609, 0.013619786812760203, -0.00169016509039246, -0.00232288652978069, -0.0061436725023182245, -0.023022139445661643, -0.030817922852286413, -0.051258730413151446, -0.0323222312927787, -0.03310625563681219, -0.04051095648689349, -0.08925657278219779, -0.09796918825736341, -0.15055821217465962, -0.1329105136068727, -0.1299762385886594, -0.17090792350189102, -0.15620260753838155, -0.18205896106319586, -0.11418565696068625, -0.1232545746496396, -0.07356228635247129, -0.03228700778542085, 0.009549569069698218, 0.07719138848761906, 0.10152359536851632, 0.1561527671427665, 0.18842932399901183, 0.197859382928335, 0.19453343725660105, 0.19210936789286565, 0.1765914553867371, 0.19522756824015558, 0.1834629153940792, 0.17683776141616084, 0.1913341786113789, 0.18709085638153858, 0.17887395984914187, 0.18666708479525765, 0.17165412322309037, 0.16441728291278784, 0.17929029547841036, 0.1519138473095778, 0.15712341532648808, 0.13846590680907087, 0.12047566454624585, 0.09872909593150139, 0.08803457793522405, 0.061698277497094584, 0.045951031549315555, 0.00979828297457177, 0.008366111860549478, -0.046385788200896866, -0.07504876928232283, -0.07280270694113325, -0.03198836871640067, -0.13459344201866588, -0.07077241587313422, -0.08758411642836511, -0.07768664911688121, -0.07634884485957126, -0.08324163893018204, -0.06345115425362062, -0.06446946891957345, -0.0663578014961909, -0.06029223565621726, -0.07593611622222576, -0.0724295367493707, -0.052451805155100784, -0.04536538454303446, -0.07357329487520319, -0.052053802256372045, -0.05239139445449025, -0.044479931620665596, -0.04853718573922218, -0.03442941659419918, -0.032176792584506, -0.04540027848076946, -0.03752649340753401, -0.02957082646017158, -0.04666342022920588, -0.044359702789283355, -0.03559409390892652, -0.031351767021255444, -0.030234479207401483, -0.04735727726197432, -0.04167236227794556, -0.02421595444840663, -0.04154432906677012, -0.03200544543962749, -0.04064082895362087, -0.021548834651897775, -0.028510940660415516, -0.02985481367230436, -0.029883752213589663, -0.032999616791579034, -0.01265699831797447, -0.019818700440062495, -0.0031207156577008175, -0.018900206098225354, -0.00869078752334074, -0.010394026040961221};
float pca_vec2[110] = {0.020230459959306724, 0.032393440359406944, 0.031162227126831157, 0.03183885161218747, 0.0121549021984097, 0.015392190404741046, 0.02303865968848993, 0.022897272541160498, 0.015792671870671925, 0.02843466894941945, 0.01387757755673238, 0.004153694208116774, 0.006985863923666282, 0.0028685956760995585, 0.0021411042047076514, 0.034851802771515075, -0.019148432278808244, -0.04625922432023216, -0.11498972106055019, -0.1491430925731177, -0.11278082764887479, -0.12916247480197174, -0.17301121808896286, -0.16868100790727295, -0.13682223933975135, -0.17287273343273524, -0.19359151591228413, -0.18926180323473793, -0.14139041295190732, -0.23511929226722567, -0.22887422229850907, -0.21123178213354563, -0.16646462514109278, -0.11124786602112087, -0.06523559509565538, -0.06590412387567783, -0.02985932424681171, -0.013806343923172372, -0.029809104057458832, -0.03627733831838511, -0.019202336321522184, -0.04706409091961876, -0.01949618017979867, -0.008713890225236417, -0.037461526252568426, 0.009997261791408675, -0.010549107370100369, 0.00021208681980647507, 0.008301012490414863, 0.03950898609237032, 0.0019915976352958063, 0.0021565296220990085, 0.014838993756734512, -0.04225045179814953, -0.07293778952369742, -0.08425918570717435, -0.114611875260066, -0.16220976281649063, -0.1738330530913924, -0.10803236293901869, -0.11579675958733997, -0.04991323318697854, -0.09429764122659028, -0.04180480591041613, 0.031289915507414084, 0.000766619457406987, -0.019595834404212215, 0.028003205463506358, 0.018767521322283993, -0.05039488540740291, 0.030424831885597928, 0.05267398950976214, 0.013837840914775756, 0.04258393339590005, 0.060722216655410687, 0.06402058942436267, 0.05954503705457992, 0.07526284339995216, 0.08743120964565748, 0.07869895886486619, 0.07215915116058982, 0.10659391173525393, 0.09302303066942594, 0.10033395297373467, 0.10118382396739457, 0.10875753883045615, 0.12154957714614328, 0.11355105272067287, 0.0978135098712936, 0.10752887305631094, 0.11750437858619354, 0.12617843528221728, 0.14169752905239769, 0.13657959208410583, 0.1205290963024467, 0.13010923114882736, 0.12791156249642321, 0.0977544303595024, 0.13233246623972936, 0.12143521070496213, 0.11192034145119373, 0.0908654238195867, 0.09353832001554055, 0.09055311446299681, 0.10063753358004167, 0.11156086501816488, 0.08241340432328277, 0.0911866227920746, 0.07779446269265246, 0.06912348675295167};
float projected_mean_vec[2] = {0.0018565564257895441, -0.044442684215368115};
float centroid1[2] = {-0.02132437532259558, -0.023096219947210922};
float centroid2[2] = {0.006327990934130111, 0.014882432359758032};
float centroid3[2] = {-0.029688768615139943, 0.01669358791430243};
float centroid4[2] = {0.0446851530036054, -0.008479800326849527};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------------------------------*/
/*---------------------------------------------------*/
/*---------------------------------------------------*/

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(MIC_INPUT, INPUT);

  for (int i = 0; i < 4; i++) {
    sample_lens[i] = run_times[i] / SAMPLING_INTERVAL;
  }

  write_pwm(0, 0);
  delay(2000); // Wait 2 seconds to put down car
  reset_blinker();
  start_listen_mode();
}

void loop(void) {
  check_encoders();
  if (timer_mode == MODE_LISTEN && re_pointer == SIZE){
    // Stop motor
    write_pwm(0, 0);
    digitalWrite(RED_LED, LOW);

    // if enveloped data is above some preset value
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*     From classify.ino     */
      /*     with more changes     */
      /*---------------------------*/
// Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i] * pca_vec1[i];
          proj2 += result[i] * pca_vec2[i];
          //proj3 += result[i] * pca_vec3[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];
      //proj3 -= projected_mean_vec[2];

      // Classification
      // Use the function 'l2_norm' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      // YOUR CODE HERE
      float distance = 0;
      for (int i = 0; i < 4; i++) {
        distance = l2_norm(proj1, proj2, centroids[i]);
        if (distance < best_dist) {
          best_dist = distance;
          best_index = i;
        }
      }


      // Check against EUCLIDEAN_THRESHOLD and execute identified command
      // YOUR CODE HERE
      if (best_dist < EUCLIDEAN_THRESHOLD) {
        drive_mode = commands[dex]; // from 0-3, inclusive
        dex = (dex + 1) % 9;
        start_drive_mode();
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    delay(2000);
    re_pointer = 0; // start recording from beginning if we don't start driving
  }

  else if (loop_mode == MODE_DRIVE && do_loop) {
    if (step_num < JOLT_STEPS) {
      write_pwm(left_jolt, right_jolt);
    }
    else {

      // Save positions because _left_position and _right_position
      // can change in the middle of one loop.
      int left_position = left_encoder.pos;
      int right_position = right_encoder.pos;

      /*---------------------------*/
      /*      CODE BLOCK CON0      */
      /*---------------------------*/

      float delta = left_position - right_position + delta_ss;
      delta = delta - delta_reference(step_num) - straight_correction(step_num);

      // Drive straight using feedback
      // Compute the needed pwm values for each wheel using delta and v_star
      int left_cur_pwm = driveStraight_left(delta);
      int right_cur_pwm = driveStraight_right(delta);
      write_pwm(left_cur_pwm, right_cur_pwm);

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }

    // Counter for how many times loop is executed since entering DRIVE MODE
    step_num++;

    if (step_num == sample_lens[drive_mode]) {
      // Completely stop and go back to listen MODE after 3 seconds
      start_listen_mode();
    }

    do_loop = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void reset_blinker(void) {
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

void start_listen_mode(void) {
  re_pointer = 0;
  write_pwm(0, 0);
  delay(3000); // 3 seconds buffer for mic cap settling
  timer_mode = MODE_LISTEN;
  setTimer(MODE_LISTEN);
}

void start_drive_mode(void) {
  timer_mode = MODE_DRIVE;
  step_num = 0;
  left_encoder.pos = 0;
  right_encoder.pos = 0;
  setTimer(MODE_DRIVE);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

#define AVG_DECAY_RATE              0.3
#define LOW_THRESH                  ((int) (0.1*4096))
#define HIGH_THRESH                 ((int) (0.4*4096))

void check_encoder(encoder_t* enc) {
  int new_val = analogRead(enc->pin);
  enc->avg = (int) (AVG_DECAY_RATE*enc->avg + (1 - AVG_DECAY_RATE)*new_val);
  if ((enc->level == LOW && HIGH_THRESH < enc->avg) ||
      (enc->level == HIGH && enc->avg < LOW_THRESH)) {
    enc->pos++;
    enc->level = !enc->level;
  }
}

void check_encoders(void) {
  check_encoder(&left_encoder);
  check_encoder(&right_encoder);
}

// Set timer for timestep; use B0 to free up all other PWM ports
void setTimer(boolean mode) {
  if (mode == MODE_LISTEN) {
    // Set the timer based on 25MHz clock
    TB0CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
    TB0CCTL0 = CCIE;
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
  }
  else if (mode == MODE_DRIVE) {
    TB0CCR0 = (unsigned int) (32.768*SAMPLING_INTERVAL); // set the timer based on 32kHz clock
    TB0CCTL0 = CCIE; // enable interrupts for Timer B
    __bis_SR_register(GIE);
    TB0CTL = TASSEL_1 + MC_1 + TACLR + ID_0;
  }
  timer_mode = mode;
}

// ISR for timestep
#pragma vector=TIMER0_B0_VECTOR    // Timer B ISR
__interrupt void Timer0_B0_ISR(void) {
  if (timer_mode == MODE_LISTEN) {
    if (re_pointer < SIZE) {
      digitalWrite(RED_LED, HIGH);
      re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
      re_pointer += 1;
    }
  }
  else if (timer_mode == MODE_DRIVE) {
    do_loop = 1;
  }
}
