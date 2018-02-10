#include "nodeimu.h"
#include <nan.h>

using namespace v8;

NodeIMU::NodeIMU() {
    settings = new RTIMUSettings("RTIMULib");
    imu = RTIMU::createIMU(settings);
	pressure = RTPressure::createPressure(settings);
	humidity = RTHumidity::createHumidity(settings);

    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        exit(1);
    }
	
    imu->IMUInit();
	if (pressure != NULL) { pressure->pressureInit(); }
	if (humidity != NULL) { humidity->humidityInit(); }

    imu->setSlerpPower((float)0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);
}

NodeIMU::~NodeIMU() {
    delete imu;
    delete settings;
	delete pressure;
	delete humidity;
}

NAN_METHOD(NodeIMU::New) {
	NodeIMU* obj = new NodeIMU();
	obj->Wrap(info.This());
	info.GetReturnValue().Set(info.This());
}

void AddRTVector3ToResult(v8::Handle<v8::Object>& result, RTVector3 data, const char* name) {
	Nan::HandleScope();

	v8::Local<v8::Object> field = Nan::New<v8::Object>();
	Nan::Set(field, Nan::New("x").ToLocalChecked(), Nan::New(data.x()));
	Nan::Set(field, Nan::New("y").ToLocalChecked(), Nan::New(data.y()));
	Nan::Set(field, Nan::New("z").ToLocalChecked(), Nan::New(data.z()));

	Nan::Set(result, Nan::New(name).ToLocalChecked(), field);
}

void PutMeasurement(const RTIMU_DATA& imuData, const bool pressure, const RTIMU_DATA& pressureData, const bool humidity, const RTIMU_DATA& humidityData, v8::Handle<v8::Object>& result) {
	Nan::HandleScope();
	
	Nan::Set(result, Nan::New("timestamp").ToLocalChecked(), Nan::New<v8::Date>(0.001 * (double)imuData.timestamp).ToLocalChecked());

	AddRTVector3ToResult(result, imuData.accel, "accel");
	AddRTVector3ToResult(result, imuData.gyro, "gyro");
	AddRTVector3ToResult(result, imuData.compass, "compass");
	AddRTVector3ToResult(result, imuData.fusionPose, "fusionPose");
	
	Nan::Set(result, Nan::New("tiltHeading").ToLocalChecked(), Nan::New(RTMath::poseFromAccelMag(imuData.accel, imuData.compass).z()));

	if (pressure) {
		Nan::Set(result, Nan::New("pressure").ToLocalChecked(), Nan::New(pressureData.pressure));
		Nan::Set(result, Nan::New("pressureValid").ToLocalChecked(), Nan::New(pressureData.pressureValid));
		Nan::Set(result, Nan::New("pressureTemperature").ToLocalChecked(), Nan::New(pressureData.temperature));
		Nan::Set(result, Nan::New("pressureTemperatureValid").ToLocalChecked(), Nan::New(pressureData.temperatureValid));
	}
	if (humidity) {
		Nan::Set(result, Nan::New("humidity").ToLocalChecked(), Nan::New(humidityData.humidity));
		Nan::Set(result, Nan::New("humidityValid").ToLocalChecked(), Nan::New(humidityData.humidityValid));
		Nan::Set(result, Nan::New("humidityTemperature").ToLocalChecked(), Nan::New(humidityData.temperature));
		Nan::Set(result, Nan::New("humidityTemperatureValid").ToLocalChecked(), Nan::New(humidityData.temperatureValid));
	}
}

class SensorWorker : public Nan::AsyncWorker {
public:
	SensorWorker(Nan::Callback *callback, RTIMU* imu, RTPressure *pressure,	RTHumidity *humidity)
		: AsyncWorker(callback), d_imu(imu), d_pressure(pressure), d_humidity(humidity) {}
	~SensorWorker() {}

	// Executed inside the worker-thread.
	// It is not safe to access V8, or V8 data structures
	// here, so everything we need for input and output
	// should go on `this`.
	void Execute() {
		if (d_imu->IMURead()) {
			d_imuData = d_imu->getIMUData();
			if (d_pressure != NULL) { d_pressure->pressureRead(d_imuData); }
			if (d_humidity != NULL) { d_humidity->humidityRead(d_imuData); }
		}
	}

	// Executed when the async work is complete
	// this function will be run inside the main event loop
	// so it is safe to use V8 again
	void HandleOKCallback() {
		Nan::HandleScope scope;

		v8::Local<v8::Object> result = Nan::New<v8::Object>();
		PutMeasurement(d_imuData, d_pressure != NULL,d_humidity != NULL, result);

		v8::Local<Value> argv[] = {
			Nan::Null(), result
		};

		callback->Call(2, argv);
	}

private:
	RTIMU* d_imu;
	RTPressure *d_pressure;
	RTHumidity *d_humidity;

	RTIMU_DATA d_imuData;
};

NAN_METHOD(NodeIMU::GetValue) {
	NodeIMU* obj = Nan::ObjectWrap::Unwrap<NodeIMU>(info.This());
	Nan::Callback *callback = new Nan::Callback(info[0].As<Function>());
	Nan::AsyncQueueWorker(new SensorWorker(callback, obj->imu, obj->pressure, obj->humidity));
}

NAN_METHOD(NodeIMU::GetValueSync) {
	NodeIMU* obj = Nan::ObjectWrap::Unwrap<NodeIMU>(info.This());
	if (obj->imu->IMURead()) {
		RTIMU_DATA imuData = obj->imu->getIMUData();
		RTIMU_DATA pressureData;
		RTIMU_DATA humidityData;
		bool pressure = (obj->pressure != NULL);
		bool humidity = (obj->humidity != NULL);
		if (pressure) { obj->pressure->pressureRead(pressureData); }
		if (humidity) { obj->humidity->humidityRead(humidityData); }

		v8::Local<v8::Object> result = Nan::New<v8::Object>();
		PutMeasurement(imuData, pressure, pressureData, humidity, humidityData, result);

		info.GetReturnValue().Set(result);
	}
}
