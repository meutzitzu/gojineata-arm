#include<AFMotor.h>

enum class Direction{ forward = 1, reverse =-1};

class Motorash
{
	AF_DCMotor* m_motor;
	byte m_id;
	float m_power;
	byte m_truepower;

	Direction m_direction;

	public:
	Motorash( byte id, Direction direction);
	~Motorash();

	void setPower( float power);
	float getPower();
	Direction getDirection();
};

class Encoderutz
{
/*	This is a heavy insult to the word "Encoderutz"
 *	as we are using potentiometers,
 *	not even multi-turn ones in order
 *	to measure the angle of the joints.
 */
	
	int m_center;
	byte m_pin;
	Direction m_direction;
	
	public:
	Encoderutz( byte id, int center = 0);
	void setCenter( int center);
	float getAngle();
};

class Jointuletz
{
	Motorash m_motor;
	Encoderutz m_encoder;

	float lower_bound;
	float upper_bound;

	public:
	Jointuletz( Motorash m_motor, Encoderutz m_encoder);

	void setPosition( float angle);
	float getPosition();
};

Motorash::Motorash( byte id, Direction direction)
{
	m_motor = new AF_DCMotor(id);
	m_id = id;
	setPower(0.0);
}

Motorash::~Motorash()
{
	delete m_motor;
}

float Motorash::getPower()
{
	return m_power;
}

Direction Motorash::getDirection()
{
	return m_direction;
}

void Motorash::setPower( float power)
{
	m_power = (int)m_direction * power;
	m_truepower = (byte)(abs( 255 * m_power));
	m_motor->setSpeed(m_truepower);
	m_motor->run( m_power > 0 ? FORWARD : BACKWARD);
}


Encoderutz::Encoderutz( byte pin, int center)
{
	m_pin = pin;
	m_center = center;
}

float Encoderutz::getAngle()
{
	return ((int)m_direction * (analogRead(m_pin)-m_center)/((350/180)*PI));
}

Motorash* motor1 = new Motorash( 3, Direction::forward);

void setup()
{
//	motor.setSpeed(255);
	Serial.begin(9600);
}

void loop()
{
	
	enum state{STOP, FWD, REV};

	state State = STOP;
	if(Serial.available()){
		char c = Serial.read();
		switch(c)
		{
			case 'q':
				motor1->setPower( 0.0);
				State = STOP;
				break;
			
			case 'w':
				motor1->setPower( 1.0);
				State = FWD;
				break;

			case 's':
				motor1->setPower(-1.0);
				State = REV;
				break;
		}
	}
	Serial.println(analogRead(A3));
	Serial.println(State);
	delay(200);
}
