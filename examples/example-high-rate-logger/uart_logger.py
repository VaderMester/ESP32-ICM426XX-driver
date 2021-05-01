
from optparse import OptionParser
import serial
import datetime
import time
import os
import matplotlib
import matplotlib.pyplot as plt

class Uart_Logger():

	def __init__(self):
		# List of possible headers list in byte format
		self.header_lsb = b'\x55'
		self.header_msb_32 = b'\xA0'
		self.header_msb_16 = b'\xA1'
		self.header_msb_8 = b'\xA2'
		# Set error to True, to check data have been streamed
		self.check_error = True
		# Log Uart bytes into a temp binary file
		self.outputFilePathBin = "temp.bin"

	def __del__(self):
		# Delete temp binary file
		os.remove(self.outputFilePathBin)

	def byte_logger(self, device, speed, time_s):
		
		start_time = time.time()
		stop_time = start_time + float(time_s)

		# Log UART during time_s
		with serial.Serial(device, speed, xonxoff=False, rtscts=True) as ser, open(self.outputFilePathBin, mode='wb') as f :
			print("Logging started. Ctrl-C to stop.") 
			try:
				while time.time() < stop_time:
					f.write((ser.read(ser.inWaiting())))
					f.flush()
			except KeyboardInterrupt:
				print("Byte logger stopped")

		f.close()

	def byte_parser(self, printer, checker, logger, plotter):
		
		name = datetime.datetime.now().strftime("%Y-%m-%dT%H.%M.%S")

		# If logger option set we create a txt file
		if logger :
			outputFilePathTxt = os.path.join(os.path.dirname(__file__), name + ".txt")
			outputFileTxt = open(outputFilePathTxt, mode='w')

		# Parse file, check error if option check and add possibility to print converted datas if option
		with open(self.outputFilePathBin, mode='rb') as f :
			print("Parsing started. Ctrl-C to stop.") 
			try :
				loop_end = False
				prev_counter = 0
				prev_counter_ext = 0
				rollover = 0
				
				# For plotting
				counter_plot = []
				data_plot = []
				odr_is_32khz = True

				while not loop_end :
					h1 = f.read(1)
					# Find first header 0x55 
					if h1 == self.header_lsb :
						h2 = f.read(1)
						# Find second header according to odr 0xAn 
						if (h2 == self.header_msb_32) or (h2 == self.header_msb_16) or (h2 == self.header_msb_8) :
							# At least one of the header list found, now get counter and raw acc x datas
							counter_byte = f.read(1)
							if counter_byte == b'':
								loop_end = True
								break # end loop now as we reached end of file

							counter = int.from_bytes(counter_byte, "little")
							
							if counter < prev_counter :
								rollover = rollover + 1
							counter_ext = rollover * (255 + 1) + counter
							prev_counter = counter
							
							if (h2 == self.header_msb_16) or (h2 == self.header_msb_8) :
								
								odr_is_32khz = False

								raw_a_x_b = f.read(2)
								raw_a_y_b = f.read(2)
								raw_a_z_b = f.read(2)
								raw_g_x_b = f.read(2)
								raw_g_y_b = f.read(2)
								raw_g_z_b = f.read(2)
								if (   raw_a_x_b == b'' 
									or raw_a_y_b == b''
									or raw_a_z_b == b''
									or raw_g_x_b == b''
									or raw_g_y_b == b''
									or raw_g_z_b == b''):
									loop_end = True
									break # end loop now as we reached end of file

								# ODR is 8 or 16 KHz, all 6-axis should be recorded
								raw_a_x = int.from_bytes(raw_a_x_b, "big", signed=True)
								raw_a_y = int.from_bytes(raw_a_y_b, "big", signed=True)
								raw_a_z = int.from_bytes(raw_a_z_b, "big", signed=True)
								raw_g_x = int.from_bytes(raw_g_x_b, "big", signed=True)
								raw_g_y = int.from_bytes(raw_g_y_b, "big", signed=True)
								raw_g_z = int.from_bytes(raw_g_z_b, "big", signed=True)

								if printer :
									print("racc", counter_ext, raw_a_x, raw_a_y, raw_a_z)
									print("rgyr", counter_ext, raw_g_x, raw_g_y, raw_g_z)

								if logger :
									outputFileTxt.write("racc %d %d %d %d\n" %(counter_ext, raw_a_x, raw_a_y, raw_a_z))
									outputFileTxt.write("rgyr %d %d %d %d\n" %(counter_ext, raw_g_x, raw_g_y, raw_g_z))

								if plotter : 
									data_plot.append([raw_a_x, raw_a_y, raw_a_z, raw_g_x, raw_g_y, raw_g_z])
							else :
								
								raw_data_b = f.read(2)
								if raw_data_b == b'':
									loop_end = True
									break # end loop now as we reached end of file

								raw_data = int.from_bytes(raw_data_b, "big", signed=True)

								# ODR is 32 KHz, only one axis is recorded
								if printer :
									print("data", counter_ext, raw_data)
								
								if logger :
									outputFileTxt.write("data %d %d \n" %(counter_ext, raw_data))

								if plotter : 
									data_plot.append(raw_data)

							# Set error variable to False as at least one data have been streamed
							self.check_error = False

							if checker :
								if (prev_counter_ext != 0) and (counter_ext - prev_counter_ext > 1) :
									print("Error with counter, previous was ", prev_counter_ext, " current is", counter_ext)
									self.check_error = True
									loop_end = True
								prev_counter_ext = counter_ext

							if plotter : 
								counter_plot.append(counter_ext)

					elif h1 == b'' :
						# No remainig data in the file, end has been reached
						loop_end = True

				if plotter :
					plt.figure(figsize=(16.8,8.0))
					if odr_is_32khz :
						plt.plot(counter_plot, data_plot)
						plt.ylabel('Raw data')
						plt.title(name)
					else :
						plt.subplot(2, 1, 1)
						plt.plot(counter_plot, [i[0] for i in data_plot], label='x')
						plt.plot(counter_plot, [i[1] for i in data_plot], label='z')
						plt.plot(counter_plot, [i[2] for i in data_plot], label='y')
						plt.ylabel('Accel data')
						plt.legend()
						plt.subplot(2, 1, 2)
						plt.plot(counter_plot, [i[3] for i in data_plot], label='x')
						plt.plot(counter_plot, [i[4] for i in data_plot], label='z')
						plt.plot(counter_plot, [i[5] for i in data_plot], label='y')
						plt.ylabel('Gyro data')  
						plt.legend()
						plt.suptitle(name)
					plt.xlabel('Counter')
					plt.show()



			except KeyboardInterrupt :
				print("Byte parser stopped")

		f.close()
		
		if (self.check_error == True) :
			print ("Error on data logged !!!")
		elif (checker and (self.check_error == False)) :
			print ("Counter checked successfully")

		if logger :
			outputFileTxt.flush()
			outputFileTxt.close()


def main():

	parser = OptionParser()
	parser.add_option("-p", "--port",
						dest="port",
						help="USB Serial COM Port ID")
	
	parser.add_option("-s", "--speed",
						dest="speed",
						type=int,
						help="Uart baudrate",
						default=3000000)

	parser.add_option("-d", "--duration", 
						dest="duration", 
						type=int,           
						help="Logging duration in second",
						default=3600)

	parser.add_option("-r", "--printer",  
						dest="printer",
						action="store_true",
						help="Once record is completed, print data",
						default=False)

	parser.add_option("-c", "--checker",
						dest="checker",
						action="store_true",
						help="Once record is completed, check counter for data losses",
						default=False)

	parser.add_option("-l", "--logger",
						dest="logger",
						action="store_true",
						help="Once record is completed, write data into a file",
						default=False)

	parser.add_option("-o", "--plotter",
						dest="plotter",
						action="store_true",
						help="Once record is completed, plot data using matplotlib",
						default=False)

	options, args = parser.parse_args()
	if not options.port:
		parser.error('Please specify COM port with `-p COMXX` or `--port COMXX`')

	print("COM port: {}".format(options.port))
	print("UART speed: {}".format(options.speed))
	print("test duration (s): {}".format(options.duration))
	print("print data: {}".format(options.printer))
	print("check counter: {}".format(options.checker))
	print("log data: {}".format(options.logger))
	print("plot data: {}".format(options.plotter))

	uart_logger = Uart_Logger()

	uart_logger.byte_logger(options.port, options.speed, options.duration)
	uart_logger.byte_parser(options.printer, options.checker, options.logger, options.plotter)

if __name__ == "__main__":
	main()
