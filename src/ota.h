const char PAGE_Update[] PROGMEM = R"=====(

<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>GNSS Time Server OTA Firmware Update</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin-top: 100px;
        }
        
        .container {
            display: inline-block;
            padding: 20px;
            border: 1px solid #ccc;
            border-radius: 10px;
            box-shadow: 0 0 10px rgba(0, 0, 0, 0.2);
        }
        
        input[type="file"],
        button {
            padding: 10px;
            margin-top: 10px;
            border: none;
            border-radius: 6px;
            cursor: pointer;
        }
        
        button {
            background-color: #0094FF;
            color: white;
        }
    </style>
</head>

<body>

    <div class="container">
        <h2>GNSS Time Server OTA Firmware Update</h2>
        <form action="/update" method="POST" enctype="multipart/form-data">
            <input type="file" name="update" accept=".bin" required>
            <br>
            <button type="submit" value="Update">Upload and Update</button>
        </form>
    </div>

</body>

</html>

)=====";
