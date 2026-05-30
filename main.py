from hpasim.parking_scenarios import write_all_scenarios


def main():
    for path in write_all_scenarios():
        print(path)


if __name__ == "__main__":
    main()
