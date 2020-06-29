from drivers.ur5e_driver import UR5e_driver

ur5e_driver = UR5e_driver()
def main():
    joints = ur5e_driver.get_joint()
    print joints

if __name__ == "__main__":
    main()