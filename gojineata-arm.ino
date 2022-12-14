#include<AFMotor.h>

enum class Direction:int{ forward = 1, reverse =-1};

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
	
	byte m_pin;
	Direction m_direction;
	
	public:
	int m_center;
	Encoderutz( byte id, int center = 0, Direction direction = Direction::forward);
	void setCenter( int center);
	float getAngle();
	int getRAW();
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
	Controlerutz* m_controller;

	float m_lower_bound;
	float m_upper_bound;
	float m_position;

	public:
	Encoderutz* m_encoder;
	Jointuletz( Motorash* motor, Encoderutz* encoder, Controlerutz* controller);
	~Jointuletz();

	void setBounds( float lower, float upper);
	void setPosition( float angle);
	void setPower( float power);
	void zero();
	void control();
	float getPosition();
	float getRAW();
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
	Serial.print("SPEED:");
	Serial.println(m_power);
	m_truepower = (byte)(abs( 255 * m_power));
	m_motor->setSpeed(m_truepower);
	m_motor->run( m_power > 0 ? FORWARD : BACKWARD);
}

Encoderutz::Encoderutz( byte pin, int center, Direction direction)
{
	m_pin = pin;
	m_center = center;
	m_direction = direction;
}

int Encoderutz::getRAW()
{
	return analogRead(m_pin);
}

float Encoderutz::getAngle()
{
	float sign = (float)static_cast<int>(m_direction);
	return ( sign * (getRAW()-m_center)*((350.0/180)*3.141592f/1024));
}

void Encoderutz::setCenter( int center)
{
	m_center = center;
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
Jointuletz::~Jointuletz()
{
	delete m_motor;
	delete m_encoder;
	delete m_controller;
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

void Jointuletz::setPower( float power)
{
	m_motor->setPower(power);
}

void Jointuletz::zero()
{
	m_encoder->setCenter(m_encoder->getRAW());
}

float Jointuletz::getPosition()
{
	return m_encoder->getAngle();
}
float Jointuletz::getRAW()
{
	return m_encoder->getRAW();
}

void Jointuletz::control()
{
	m_motor->setPower(m_controller->getControlPower(m_position - m_encoder->getAngle()));
}

Motorash* motor1 = new Motorash( 3, Direction::forward);
Encoderutz* enc1 = new Encoderutz(A3, 512, Direction::reverse);
Controlerutz* ctrl1 = new Controlerutz( 15.0, 0.5, 5.0);

Jointuletz* forearm = new Jointuletz(motor1, enc1, ctrl1);

	enum state{STOP, MANUAL, AUTO};
	float position = 0.0;
	float power = 0.0;
	state State = STOP;

void setup()
{
//	motor.setSpeed(255);
	Serial.begin(9600);
	forearm->zero();

}

void loop()
{
	
	if(Serial.available()){
		char c = Serial.read();
		switch(c)
		{
			case '\t':
				if (State == AUTO)
				{
					State = MANUAL;
				}
				else if (State == MANUAL)
				{
					State = AUTO;
				}
				break;

			case 'd':
				if (State == AUTO)
				{
					position += 0.05;
				}
				if (State == MANUAL)
				{
					power = 1.0;
				}
				break;

			case 'a':
				if (State == AUTO)
				{
					position -= 0.05;
				}
				if (State == MANUAL)
				{
					power =-1.0;
				}
				break;
			case 's':
				if (State == AUTO)
				{
					position = 0.0;
				}
				if (State == MANUAL)
				{
					power = 0.0;
				}
				break;
			case 'q':
				State = STOP;
				break;
			case 'w':
				State = MANUAL;
				break;
			case 'e':
				forearm->zero();
				break;
		}
	}
	forearm->setPosition(position);	
	switch(State)
	{
		case STOP:
			forearm->setPower(0.0);
		break;
		case MANUAL:
			forearm->setPower(power);
		break;
		case AUTO:
			forearm->control();
		break;
	}
	Serial.println(forearm->getRAW());
	Serial.print("ACTUAL\t");
	Serial.println(forearm->getPosition());
	Serial.print("TARGET\t");
	Serial.println(position);
	Serial.println(State);
	delay(200);
}
