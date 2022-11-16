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

class Controlerutz
{
	float m_kp;
	float m_ki;
	float m_kd;

	float m_P;
	float m_I;
	float m_D;

	public:
	Controlerutz( float p, float i, float d);
	float getControlPower(float error);
};

class Jointuletz
{
	Motorash* m_motor;
	Encoderutz* m_encoder;
	Controlerutz* m_controller;

	float m_lower_bound;
	float m_upper_bound;
	float m_position;

	public:
	Jointuletz( Motorash* motor, Encoderutz* encoder, Controlerutz* controller);

	void setBounds( float lower, float upper);
	void setPosition( float angle);
	void control();
	float getPosition();
};


class Robotzel
{
	//Joint** joints;
};

Motorash::Motorash( byte id, Direction direction)
{
	m_motor = new AF_DCMotor(id);
	m_id = id;
	setPower(0.0);
	m_direction = direction;
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
	m_power = (int)(static_cast<int>(m_direction) * power);
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

Controlerutz::Controlerutz( float kp, float ki, float kd)
{
	m_kp = kp;
	m_ki = ki;
	m_kd = kd;

	m_P = 0.0;
	m_I = 0.0;
	m_D = 0.0;
}


float Controlerutz::getControlPower( float error)
{
	m_D = error - m_P;
	m_P = error;
	m_I += error;

	return m_kp*m_P + m_ki*m_I + m_kd*m_D;
}

Jointuletz::Jointuletz( Motorash* motor, Encoderutz* encoder, Controlerutz* controller)
{
	m_motor = motor;
	m_encoder = encoder;
	m_controller = controller;
	m_upper_bound = 0.0;
	m_lower_bound = 0.0;
	m_position = 0.0;
}

void Jointuletz::setBounds(float lower, float upper)
{
	m_upper_bound = upper;
	m_lower_bound = lower;
}

void Jointuletz::setPosition( float position)
{
	m_position = position;
}

void Jointuletz::control()
{
	m_motor->setPower(m_controller->getControlPower(m_position - m_encoder->getAngle()));
}

Motorash* motor1 = new Motorash( 1, Direction::forward);
Encoderutz* enc1 = new Encoderutz(A1, 512);
Controlerutz* ctrl1 = new Controlerutz( 1.0, 1.0, 1.0);

Jointuletz* forearm = new Jointuletz(motor1, enc1, ctrl1);

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
				forearm->setPosition( 0.0);
				State = STOP;
				break;
			
			case 'a':
				forearm->setPosition( 0.2);
				State = FWD;
				break;

			case 'z':
				forearm->setPosition(-0.2);
				State = REV;
				break;
		}
	}
	forearm->control();
	delay(200);
}
