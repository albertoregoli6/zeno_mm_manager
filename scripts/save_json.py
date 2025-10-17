import asyncio
import websockets
import json
import datetime
import os

# Cartella dove salvare i file JSON
SAVE_DIR = "/home/alberto/zeno_ws/src/zeno_mission_manager/json/"
os.makedirs(SAVE_DIR, exist_ok=True)


async def client():
    uri = "ws://192.168.25.11:8765"  # change to your server IP if remote
    async with websockets.connect(uri) as websocket:
        print("🔗 Connected to server!")

        while True:
            try:
                message = await websocket.recv()
                print(f"📩 Received raw message: {message}")

                try:
                    json_file = json.loads(message)
                    filename = os.path.join(SAVE_DIR, f"niasca.json")

                    # Salva il file
                    with open(filename, "w", encoding="utf-8") as f:
                        json.dump(json_file, f, indent=2, ensure_ascii=False)

                    print(f"💾 File salvato: {filename}")

                except json.JSONDecodeError:
                    print("⚠️ Errore: messaggio non è JSON valido")
                    continue

            except websockets.exceptions.ConnectionClosedOK:
                print("🔌 Connection closed normally by server.")
                break
            except websockets.exceptions.ConnectionClosedError:
                print("❌ Connection closed with error.")
                break

if __name__ == "__main__":
    asyncio.run(client())
