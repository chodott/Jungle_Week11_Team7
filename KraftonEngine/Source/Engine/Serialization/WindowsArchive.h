#pragma once

#include "Archive.h"
#include "Platform/Paths.h"
#include <fstream>
#include <string>
#include <iostream>

class FWindowsBinWriter : public FArchive
{
private:
	std::ofstream FileStream;

public:
	FWindowsBinWriter(const std::string& FilePath)
	{
		bIsSaving = true; // 나는 '쓰기' 전용이다!
		FileStream.open(FPaths::ToWide(FilePath), std::ios::binary);
	}

	~FWindowsBinWriter() override
	{
		if (FileStream.is_open()) FileStream.close();
	}

	// 파일이 정상적으로 열렸는지 확인
	bool IsValid() const { return FileStream.is_open() && FileStream.good(); }

	void Serialize(void* Data, size_t Num) override
	{
		if (FileStream.is_open() && Num > 0)
		{
			// 하드 디스크에 데이터를 씁니다.
			FileStream.write(static_cast<const char*>(Data), Num);
		}
	}
};

class FWindowsBinReader : public FArchive
{
private:
	std::ifstream FileStream;
	bool          bReadPastEnd = false;

public:
	FWindowsBinReader(const std::string& FilePath)
	{
		bIsLoading = true; // 나는 '읽기' 전용이다!
		FileStream.open(FPaths::ToWide(FilePath), std::ios::binary);
	}

	~FWindowsBinReader() override
	{
		if (FileStream.is_open()) FileStream.close();
	}

	// 파일이 정상적으로 열렸는지 확인 (읽기 전 오픈 체크용).
	bool IsValid() const { return FileStream.is_open() && FileStream.good(); }

	// 역직렬화가 실제로 실패했는지 확인 (읽기 후 검증용).
	// 파일 끝까지 정확히 읽으면 MSVC가 eofbit를 세워 good()이 false가 되지만
	// 그건 정상이다. 진짜 에러는 (1) 파일을 못 엶, (2) 스트림 손상(bad),
	// (3) 요청보다 적게 읽힘(파일이 잘렸거나 포맷이 어긋나 EOF를 넘어 읽음)뿐이다.
	bool HadError() const { return !FileStream.is_open() || FileStream.bad() || bReadPastEnd; }

	void Serialize(void* Data, size_t Num) override
	{
		if (FileStream.is_open() && Num > 0)
		{
			// 하드 디스크에서 데이터를 읽어옵니다.
			FileStream.read(static_cast<char*>(Data), Num);
			if (static_cast<size_t>(FileStream.gcount()) != Num)
			{
				bReadPastEnd = true;
			}
		}
	}
};
