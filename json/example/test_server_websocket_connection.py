import asyncio
import websockets

async def handler(websocket):
    client_ip, client_port = websocket.remote_address
    print(f"🔌 Nuova connessione da {client_ip}:{client_port}")

    try:
        async for message in websocket:
            print(f"📩 Messaggio ricevuto da {client_ip}:{client_port}: {message}")

    except websockets.exceptions.ConnectionClosed:
        print(f"❌ Connessione chiusa da {client_ip}:{client_port}")

async def main():
    async with websockets.serve(handler, "0.0.0.0", 8765):
        print("✅ WebSocket server in ascolto su ws://0.0.0.0:8765")
        await asyncio.Future()  # rimane attivo

if __name__ == "__main__":
    asyncio.run(main())
