# read_average.py

def calculate_average_from_txt(filename):
    values = []

    with open(filename, "r", encoding="utf-8") as file:
        for line in file:
            line = line.strip()

            # 跳过空行和注释行
            if not line or line.startswith("#"):
                continue

            # 按空格/Tab 分割
            parts = line.split()

            # 你的格式是：time value
            if len(parts) >= 2:
                value = float(parts[1])   # 取第二列
                values.append(value)

    if len(values) == 0:
        print("No valid data found.")
        return None

    average = sum(values) / len(values)
    return average


filename = "C:\\01_Workspace\\RM\\dart\\DartADC_G474\\DataRecord\\osc00026.txt"

avg = calculate_average_from_txt(filename)

if avg is not None:
    print(f"Average value = {avg}")