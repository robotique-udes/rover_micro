import random


def display_main_menu():
  print("\n== Reactor Control Panel Diagnostic Menu ==")
  print("Send 1 2 or 3 to select menu items")
  print("1. Option 1: Sub-Menu with Options")
  print("2. Option 2: Just an Option")
  print("3. Option 3: Data Values and Nested Menus")


def display_option1_submenu():
  print("\nOption 1 Submenu:")
  print("1. Turn On Option 1")
  print("2. Turn Off Option 1")
  print("3. Back to Main Menu")


def display_option3_submenu():
  print("\nOption 3 Submenu:")
  print("1. Just Option 1")
  print("2. Just Option 2")
  print("3. Data Values")
  print("4. Back to Main Menu")


def get_user_choice():
  choice = input("Enter your choice: ")
  return choice.strip().lower()


def option1():
  while True:
    display_option1_submenu()
    sub_choice = get_user_choice()

    if sub_choice == '1':
      print("Option 1 is ON.")
      # Add your logic for Suboption 1-1 here
    elif sub_choice == '2':
      print("Option 1 is OFF.")
      # Add your logic for Suboption 1-2 here
    elif sub_choice == '3':
      break
    else:
      print("Invalid choice. Please select a valid option (1, 2, or 3).")


def option2():
  print("You selected Option 2.")
  # Add your logic for Option 2 here


def suboption3_1():
  print("You selected Suboption 3-1.")
  # Add your logic for Suboption 3-1 here


def suboption3_2():
  while True:
    display_option3_submenu()
    sub_choice = get_user_choice()

    if sub_choice == '1':
      print("You selected Suboption 3-1.")
      # Add your logic for Suboption 3-2-1 here
    elif sub_choice == '2':
      print("You selected Suboption 3-2.")
      # Add your logic for Suboption 3-2-2 here
    elif sub_choice == '3':
      print("Entering sub-submenu for Suboption 3-2...")
      while True:
        print("Sub-submenu for Suboption 3-2:")
        print("1. Read Data Value A")
        print("2. Read Data Value B")
        print("3. Read Data Value C")
        print("4. Back to Option 3 Submenu")
        sub_sub_choice = get_user_choice()

        if sub_sub_choice == '1':
          print("Value 3-1: {}".format(random.randint(1, 255)))
        elif sub_sub_choice == '2':
          print("Value 3-2: {}".format(random.randint(255, 1024)))
        elif sub_sub_choice == '3':
          print("Value 3-2: {}".format(random.sample(['UP', 'DOWN'], 1)[0]))
        elif sub_sub_choice == '4':
          break
        else:
          print("Invalid choice. Please select a valid option (1, 2, or 3).")
    elif sub_choice == '4':
      break
    else:
      print("Invalid choice. Please select a valid option (1, 2, 3, or 4).")


def main():
  while True:
    display_main_menu()
    choice = get_user_choice()

    if choice == '1':
      option1()
    elif choice == '2':
      option2()
    elif choice == '3':
      suboption3_2()
    elif choice == '4':
      print("Goodbye!")
      break
    else:
      print("Invalid choice. Please select a valid option (1, 2, 3, or 4).")


if __name__ == "__main__":
  main()
