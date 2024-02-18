class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.posteri_estimate = [[0.0]]
        self.posteri_error_estimate = [[1.0]]

    def matrix_addition(self, matrix1, matrix2):
        result = []
        for i in range(len(matrix1)):
            row = []
            for j in range(len(matrix1[0])):
                row.append(matrix1[i][j] + matrix2[i][j])
            result.append(row)
        return result

    def matrix_multiplication(self, matrix1, matrix2):
        result = []
        for i in range(len(matrix1)):
            row = []
            for j in range(len(matrix2[0])):
                val = 0
                for k in range(len(matrix1[0])):
                    val += matrix1[i][k] * matrix2[k][j]
                row.append(val)
            result.append(row)
        return result

    def update(self, measurement):
        # Prediction update
        priori_estimate = self.posteri_estimate
        priori_error_estimate = self.matrix_addition(self.posteri_error_estimate, self.process_variance)

        # Measurement update
        blending_factor = self.matrix_multiplication(priori_error_estimate, [[1 / (priori_error_estimate[0][0] + self.measurement_variance[0][0])]])
        innovation = [[measurement[0] - priori_estimate[0][0]]]
        self.posteri_estimate = self.matrix_addition(priori_estimate, self.matrix_multiplication(blending_factor, innovation))
        self.posteri_error_estimate = self.matrix_multiplication([[1 - blending_factor[0][0]]], priori_error_estimate)

    def predict(self):
        return self.posteri_estimate


process_variance = [[1e-5]]
measurement_variance = [[0.1]]

kf = KalmanFilter(process_variance, measurement_variance)

# sensor data
sensor_data = [[0.1], [0.2], [0.3], [0.4], [0.5]]

# Perform prediction
predictions = []
for data in sensor_data:
    kf.update(data)
    predictions.append(kf.predict())

print("Predictions:", predictions)
