from controller import Controller


def main():
	controller = Controller()
	controller.up(1.5)
	controller.forward(5)
	controller.clockwise(90)
	controller.land()

if __name__ == '__main__':
	main()