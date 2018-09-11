import readline

readline.parse_and_bind('tab: complete')

running = True
while running:
    try:
        s = raw_input(">> ").strip()
    except (EOFError, KeyboardInterrupt) as e:
        running = False
        print('\nShutting down...')